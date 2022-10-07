#include "udp_link.hpp"
#include <vanetza/access/data_request.hpp>
#include <vanetza/net/ethernet_header.hpp>
#include <boost/asio/ip/multicast.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/buffers_iterator.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/dispatch.hpp>
#include <iostream>
#include <future>
#include <boost/process/io.hpp>
#include <boost/algorithm/hex.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind/bind.hpp>
#include <boost/asio/placeholders.hpp>
#include <vector>
#include <libserial/SerialStream.h>
#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>


namespace ip = boost::asio::ip;
using namespace vanetza;
using namespace boost::placeholders;

typedef struct{
    ByteBuffer::iterator short_name_inicio;
    ByteBuffer::iterator frame_inicio;
    size_t max_size;
    bool defectuosa;
}TFrameInfo;

std::vector<uint8_t> uuid {0xB3, 0x72, 0xef, 0x6a, 0x22, 0xa2, 0xe8, 0x80, 0x2b, 0x49, 0x0f, 0xed, 0x01, 0x00, 0x51, 0xe7};
std::vector<uint8_t> short_name {0x49, 0x54, 0x53, 0x00};

#define UUID_VECTOR ByteBuffer(uuid)
#define SHORT_NAME ByteBuffer(short_name)
#define HEADERS 4
#define ADV_DATA 238
#define TOTAL_LENGTH 251


//Compruebo si tengo bytes almacenados de recepciones anteriores. 
//En un buffer junto los bytes recibidos de antes (si los hubiera) y los de ahora

//Busco todas las ocurrencias de la cadena "ITS\n"


 void contains(ByteBuffer& buffer, const ByteBuffer& short_name, std::vector <TFrameInfo> &v)
{
    TFrameInfo t;
    //si no se encuentra el short name search() devuelve un puntero al final del buffer
    ByteBuffer::iterator begin = buffer.begin();
    ByteBuffer::iterator end = buffer.end();
    ByteBuffer::iterator it = std::search(buffer.begin(), buffer.end(), short_name.begin(), short_name.end());

    while(it != buffer.end()){
        t.short_name_inicio = it;
        t.frame_inicio = it + SHORT_NAME.size() + HEADERS + 1;
        t.max_size = *(t.frame_inicio - 1);
        v.push_back(t);
        it++;
        it = std::search(it, buffer.end(), short_name.begin(), short_name.end());
    }

    std::cout<<"Búsqueda finalizada. Se han encontrado "<<v.size()<<" posibles tramas"<<std::endl;
    
    //Compruebo si hay alguna defectuosa
    if(v.size() > 0){
        size_t i = 0;
        while( i < v.size() ){
            printf("Tamaño de la trama: %ld \n",v.at(i).max_size);

            if((v.size() > 1) && (i <= (v.size() - 2))){
                //Se define defectuosa como una trama que no llega completa sin posibilidad de recuperarla en la siguiente recepción de bytes
                //Si solo se recibe una única trama existe la posibilidad de recuperar el resto a continuación
                //Si se recibe más de una trama la última no tiene sentido comprobarla por el mismo motivo
                printf("Distancia entre la trama y la siguiente (en caso de que la haya): %ld\n",std::distance(v.at(i).frame_inicio, v.at(i+1).frame_inicio));
                
                if((std::distance(v.at(i).frame_inicio, v.at(i+1).frame_inicio) < TOTAL_LENGTH)){ //|| (*(v.at(i).frame_inicio) == 0xff)???????
                    printf("Defectuosa, no se recibe completa\n");
                    v.at(i).defectuosa = true;
                }else{
                    printf("No defectuosa, se recibe completa\n");
                    v.at(i).defectuosa = false;
                }
            }else{
                printf("Se considera no defectuosa por ahora\n");
                v.at(i).defectuosa = false;
            }

            if(v.at(i).max_size <= 0){
                printf("Tamaño incorrecto, trama defectuosa!\n");
                v.at(i).defectuosa = true;
            }

            i++;
        }
    }

}

void mostrar(ByteBuffer::iterator inicio, ByteBuffer::iterator final){
    ByteBuffer::iterator aux = inicio;
    while(aux != final){
        printf("%02x ", *aux);
        aux++;
    }
    std::cout<<std::endl;
}


void setDTR(boost::asio::serial_port &serial){
    int data = TIOCM_DTR; 
    int fd = serial.native_handle();    
    int err = ioctl(fd, TIOCMBIS, &data);
    if(err == -1){
        std::cerr<<"Error setDTR"<<std::endl;
    }
}

void set(boost::asio::serial_port &serial){
    int status;
    int fd = serial.native_handle();
    ioctl(fd, TIOCMGET, &status);
    status |= (TIOCM_DTR); 
    int err = ioctl(fd, TIOCMBIC, &status);
    if(err == -1){
        std::cerr<<"Error setDTR"<<std::endl;
    }
}


int get(boost::asio::serial_port &serial){
    int status = 0;
    int fd = serial.native_handle();
    ioctl(fd, TIOCMGET, &status);
    return status & TIOCM_DTR;
}

UdpLink::UdpLink(boost::asio::io_service &io_service, const ip::udp::endpoint &endpoint, const std::string& dev_tx, const std::string& dev_rx) : multicast_endpoint_(endpoint),
                                                                                           tx_socket_(io_service), rx_socket_(io_service),
                                                                                           rx_buffer_(1024, 0x00),
                                                                                           bytes_stored_(0),
                                                                                           input_stream_(io_service, STDIN_FILENO),
                                                                                           serial_stream_tx_(dev_tx),
                                                                                           serial_rx_(io_service, dev_rx)
{
    tx_socket_.open(multicast_endpoint_.protocol());

    rx_socket_.open(multicast_endpoint_.protocol());
    rx_socket_.set_option(ip::udp::socket::reuse_address(true));

    // To bind a UDP socket when receiving multicast means to specify an address and port from which to receive data
    rx_socket_.bind(multicast_endpoint_);
    rx_socket_.set_option(ip::multicast::enable_loopback(false));
    rx_socket_.set_option(ip::multicast::join_group(multicast_endpoint_.address()));

    serial_stream_tx_.SetDTR(true); //true = high, false = low

    if(serial_stream_tx_.GetDTR()){
        printf("DTR high\n");
    }else{
        printf("DTR low\n");
    }
    serial_stream_tx_.SetBaudRate(BaudRate::BAUD_115200);
    serial_stream_tx_.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);
    serial_stream_tx_.SetParity(Parity::PARITY_DEFAULT); //NONE
    serial_stream_tx_.SetStopBits(StopBits::STOP_BITS_1);
    serial_stream_tx_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    
    boost::system::error_code ec;
    serial_rx_.set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial_rx_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_rx_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_rx_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_rx_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::hardware), ec);
    

    aux_rx_buffer_.clear();
    comp_rx_buffer_.clear();

    do_receive();
}


void UdpLink::indicate(IndicationCallback cb)
{    
    callback_ = cb;
}



void UdpLink::process_frame(ByteBuffer::iterator inicio, ByteBuffer::iterator final){
    ByteBuffer its_frame(inicio, final);

    printf("UdpLink::process_frame\n");
    CohesivePacket packet(std::move(its_frame), OsiLayer::Link); // se asignan los bits a la capa de enlace
    if (packet.size(OsiLayer::Link) < EthernetHeader::length_bytes)
    {
        printf("packet.size(OsiLayer::Link) %ld\n", packet.size(OsiLayer::Link));
        printf("EthernetHeader::length_bytes %ld\n", EthernetHeader::length_bytes);
        printf("Dropped UDP packet too short to contain Ethernet header\n");
    }else{
        // Set boundary of layer data. Data beyond boundary belongs to next upper layer afterwards.
        std::cout<<"Se sube\n";
        packet.set_boundary(OsiLayer::Link, EthernetHeader::length_bytes); // selecciona la trama Ethernet contenida en UDP?
        auto link_range = packet[OsiLayer::Link];
        EthernetHeader eth = decode_ethernet_header(link_range.begin(), link_range.end());
        if (callback_)
        {
            printf("llamada al callback\n");
            callback_(std::move(packet), eth);
        }else{
            printf("No llamada al callback\n");
        }
    }
}


//Creo un nuevo buffer. El objetivo es que no cambie entre llamadas sucesivas al callback
//mi idea era hacer la variable local y hacerla volatile pero los métodos definidos para std::vector<> no tienen creados
//overloads para volatile

//Cuando se ejecuta el callback tengo que comprobar si el buffer auxiliar contiene

void UdpLink::serial_handler(boost::system::error_code ec, std::size_t length){
    if (!ec){

        std::vector <TFrameInfo> frames = std::vector <TFrameInfo>();
        frames.clear();

        std::cout << "***************** Se recibe una trama a través del puerto serie ********************** \n" << std::endl;
        std::cout <<"Número de bytes leidos: "<<length<<std::endl; 
        printf("Bytes recibidos:\n");
        mostrar(rx_buffer_.begin(), rx_buffer_.begin()+length); 

        //En el buffer completo introduzco la trama incompleta anterior (si la hay) y todos los datos recibidos
        comp_rx_buffer_.assign(aux_rx_buffer_.begin(), aux_rx_buffer_.end());  
        comp_rx_buffer_.insert(comp_rx_buffer_.end(), rx_buffer_.begin(), rx_buffer_.begin()+length);


        printf("Buffer en el que se van a buscar tramas:\n");
        mostrar(comp_rx_buffer_.begin(), comp_rx_buffer_.end());  

        contains(comp_rx_buffer_, SHORT_NAME, frames);  //devuelve un vector con el inicio al short name de todas las tramas recibidas
        printf("Número de tramas encontradas: %ld\n", frames.size());

        if(!frames.empty()){
            size_t distancia;
            bool completa;

            for(int i = 0; i < frames.size(); i++){

                printf("Analizando trama ITS \n");
                printf("Inicio: %02x  \n", *(frames.at(i).frame_inicio));
                printf("Tamaño de la trama completa: %ld \n", frames.at(i).max_size);
                printf("Defectuosa: %d \n", frames.at(i).defectuosa);

                if(frames.at(i).defectuosa){
                    printf("Defectuosa! Se descarta\n");
                }else{
                    printf("No defectuosa!\n");
                    if(i == (frames.size() -1)){ //última o primera (si solo hay una) -> las únicas que pueden tener problemas de completitud
                        printf("Comprobar si está o no completa\n");
                        distancia = std::distance(frames.at(i).frame_inicio, comp_rx_buffer_.end());
                        printf("Distancia con el final: %ld \n", distancia);
                        if(distancia < ADV_DATA -1){
                            //La trama no está completa 
                            //Almaceno lo recibido en un buffer auxiliar
                            printf("Trama no completa (2)\n");
                            bytes_stored_ = std::distance(frames.at(i).frame_inicio, comp_rx_buffer_.end());
                            printf("Bytes stored: %d\n",bytes_stored_);
                            aux_rx_buffer_.insert(aux_rx_buffer_.end(), frames.at(i).short_name_inicio, comp_rx_buffer_.end()); 
                            completa = false;

                        }else if(frames.at(i).frame_inicio >= comp_rx_buffer_.end()){
                            //Ha recibido el short name pero no el inicio como tal de la trama ITS
                            printf("Trama no completa (3)\n");
                            bytes_stored_ = 0;
                            printf("Bytes stored: %d\n",bytes_stored_);
                            aux_rx_buffer_.insert(aux_rx_buffer_.end(), frames.at(i).short_name_inicio, comp_rx_buffer_.end()); 
                            completa = false;

                        }else{
                            printf("Se supone completa\n");
                            completa = true;
                        }

                    }else{ //resto
                        distancia = std::distance(frames.at(i).frame_inicio, frames.at(i+1).frame_inicio);
                        printf("Distancia con la siguiente %ld\n", distancia);
                        completa = true;
                    }

                    if(completa){
                        printf("Se procesa la trama en la pila ITS\n");
                        aux_rx_buffer_.clear();
                        bytes_stored_ = 0;
                        process_frame(frames.at(i).frame_inicio, frames.at(i).frame_inicio+frames.at(i).max_size);
                    }
                }
            }

        }else{
            printf("No se ha encontrado ninguna trama. Se almacena para la próxima vez que se reciban datos\n");
            bytes_stored_ += length;
            aux_rx_buffer_.insert(aux_rx_buffer_.end(), comp_rx_buffer_.begin(),comp_rx_buffer_.begin()+bytes_stored_);
        }

        rx_buffer_.assign(1024, 0x00);
        comp_rx_buffer_.clear();
        frames.clear();

        std::cout << "****************************************************************************\n" << std::endl;

        serial_rx_.async_read_some(boost::asio::buffer(rx_buffer_), boost::bind(&UdpLink::serial_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
}

void UdpLink::do_receive()
{

    serial_rx_.async_read_some(boost::asio::buffer(rx_buffer_), boost::bind(&UdpLink::serial_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

    input_stream_.async_read_some(boost::asio::buffer(rx_buffer_), 
        [this](boost::system::error_code ec, std::size_t length){
            if (!ec)
            {
                std::cout << "***************** Se recibe una trama a través de stdin ***********************" << std::endl;
                std::string frame(rx_buffer_.begin(), rx_buffer_.begin() + length - 1); // almaceno lo que se ha leido en una cadena para realizar el procesamiento
                //ByteBuffer buffer(rx_buffer_.begin(), rx_buffer_.begin() + (length - 1) / 2);
                ByteBuffer buffer((length - 1) / 2);
                buffer.clear();
                int tama = 0;
                for (int i = 0; i < frame.length(); i += 2)
                {
                    tama++;
                    std::string byteString = frame.substr(i, 2);
                    uint8_t byte = strtol(byteString.c_str(), NULL, 16);
                    buffer.push_back(byte);
                }

                // constructor de CohesivePakcet -> Create packet from buffer and assign all bytes to given layer
                CohesivePacket packet(std::move(buffer), OsiLayer::Link); // se asignan los bits a la capa de enlace
                // packet.size() -> Get size of a single layer in packet
                if (packet.size(OsiLayer::Link) < EthernetHeader::length_bytes)
                {
                    std::cerr << "Dropped UDP packet too short to contain Ethernet header\n";
                }
                else
                {
                    std::cout<<"Set boundary"<<std::endl;
                    // Set boundary of layer data. Data beyond boundary belongs to next upper layer afterwards.
                    packet.set_boundary(OsiLayer::Link, EthernetHeader::length_bytes); // selecciona la trama Ethernet contenida en UDP?
                    auto link_range = packet[OsiLayer::Link];
                    EthernetHeader eth = decode_ethernet_header(link_range.begin(), link_range.end());
                    if (callback_)
                    {
                        std::cout<<"callback_"<<std::endl;
                        callback_(std::move(packet), eth);
                    }
                }
                do_receive();
                std::cout << "********************************************************************************" << std::endl;
            }
        });
    
    rx_socket_.async_receive_from(boost::asio::buffer(rx_buffer_), rx_endpoint_,
        [this](boost::system::error_code ec, std::size_t length){
            if (!ec)
            {
                std::cout << "***************** Se recibe una trama a través del socket **********************" << std::endl;
                //Se va a recibir la trama generada por el propio host porque el socket el multicast, 
                //el socket de recepción recibe la trama transmitida por el socket de transmisión. Es algo que está contemplado, 
                //luego se comprueba si el origen es el propio host para que no llegue a capas superiores
                ByteBuffer buffer(rx_buffer_.begin(), rx_buffer_.begin() + length);
                CohesivePacket packet(std::move(buffer), OsiLayer::Link); // se asignan los bits a la capa de enlace

                if (packet.size(OsiLayer::Link) < EthernetHeader::length_bytes)
                {
                    std::cerr << "Dropped UDP packet too short to contain Ethernet header\n";
                }
                else
                {
                    // Set boundary of layer data. Data beyond boundary belongs to next upper layer afterwards.
                    packet.set_boundary(OsiLayer::Link, EthernetHeader::length_bytes); // selecciona la trama Ethernet contenida en UDP?
                    auto link_range = packet[OsiLayer::Link];
                    EthernetHeader eth = decode_ethernet_header(link_range.begin(), link_range.end());
                    if (callback_)
                    {
                        callback_(std::move(packet), eth);
                    }
                }
                do_receive();
                std::cout << "********************************************************************************" << std::endl;
            }
        });
}

void UdpLink::request(const access::DataRequest &request, std::unique_ptr<ChunkPacket> packet)
{
    printf("**UdpLink::request**\n");

    packet->layer(OsiLayer::Link) = create_ethernet_header(request.destination_addr, request.source_addr, request.ether_type);

    std::array<boost::asio::const_buffer, layers_> const_buffers;
    for (auto &layer : osi_layer_range<OsiLayer::Link, OsiLayer::Application>())
    {
        const auto index = distance(OsiLayer::Link, layer);
        packet->layer(layer).convert(tx_buffers_[index]);
        const_buffers[index] = boost::asio::buffer(tx_buffers_[index]);
    }

    // Empleo tx_buffers_, un array de tamaño 6 formado por elementos de clase ByteBuffer. ByteByffer es un vector de uin8_t.
    std::cout << "Trama Ethernet generada" << std::endl;
    printf("Tamaño de la cabecera de la trama: %lu\n", tx_buffers_[0].size());
    printf("Tamaño del payload de la trama: %lu\n", tx_buffers_[1].size());
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < tx_buffers_[i].size(); j++)
        {
            printf("%02x", tx_buffers_[i].at(j));
        }
    }
    printf("\n");
    printf("Envío por socket\n");
    tx_socket_.send_to(const_buffers, multicast_endpoint_);
    printf("Finaliza el envío por socket\n");


    uint8_t tama = const_buffers[0].size() + const_buffers[1].size();
    serial_stream_tx_.write((char*) &tama, sizeof(uint8_t));
    serial_stream_tx_.write((const char*) const_buffers[0].data(), const_buffers[0].size());
    serial_stream_tx_.write((const char*) const_buffers[1].data(), const_buffers[1].size());

    std::cout << "********************************************************************************\n"
              << std::endl;
}


