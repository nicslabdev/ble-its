#include "uart_link_rx.hpp"
#include "log.hpp"
#include <vanetza/access/data_request.hpp>
#include <vanetza/net/ethernet_header.hpp>
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


using namespace vanetza;
using namespace boost::placeholders;
using namespace log_module;

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



UartLink_rx::UartLink_rx(boost::asio::io_service &io_service, const std::string& dev_rx) : rx_buffer_(1024, 0x00),
                                                                                           bytes_stored_(0),
                                                                                           serial_rx_(io_service, dev_rx)
{
    
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


//Compruebo si tengo bytes almacenados de recepciones anteriores. 
//En un buffer junto los bytes recibidos de antes (si los hubiera) y los de ahora
//Busco todas las ocurrencias de la cadena "ITS"
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

    std::cout<<"Search finished. "<<v.size()<<" possible frames found."<<std::endl;
    
    //Compruebo si hay alguna defectuosa
    printf("Looking for faulty frames...");
    if(v.size() > 0){
        size_t i = 0;
        while( i < v.size() ){

            if((v.size() > 1) && (i <= (v.size() - 2))){
                //Se define defectuosa como una trama que no llega completa sin posibilidad de recuperarla en la siguiente recepción de bytes
                //Si solo se recibe una única trama existe la posibilidad de recuperar el resto a continuación
                //Si se recibe más de una trama la última no tiene sentido comprobarla por el mismo motivo
                print_log("Distance between frames: "+std::distance(v.at(i).frame_inicio, v.at(i+1).frame_inicio));
                
                if((std::distance(v.at(i).frame_inicio, v.at(i+1).frame_inicio) < TOTAL_LENGTH)){ //|| (*(v.at(i).frame_inicio) == 0xff)???????
                    print_log("Faulty, incomplete!");
                    v.at(i).defectuosa = true;
                }else{
                    print_log("Complete!");
                    v.at(i).defectuosa = false;
                }
            }else{
                print_log("Correct!");
                v.at(i).defectuosa = false;
            }

            if(v.at(i).max_size <= 0){
                print_log("Tamaño incorrecto, trama defectuosa!");
                v.at(i).defectuosa = true;
            }

            i++;
        }
    }

}


void UartLink_rx::indicate(IndicationCallback cb)
{    
    callback_ = cb;
}



void UartLink_rx::process_frame(ByteBuffer::iterator inicio, ByteBuffer::iterator final){
    ByteBuffer its_frame(inicio, final);

    print_log("**UartLink_rx::process_frame**");
    CohesivePacket packet(std::move(its_frame), OsiLayer::Link); // se asignan los bits a la capa de enlace
    if (packet.size(OsiLayer::Link) < EthernetHeader::length_bytes)
    {
        std::string log = "packet.size(OsiLayer::Link) " +std::to_string(packet.size(OsiLayer::Link));
        print_log(log);
        log = "EthernetHeader::length_bytes "+ std::to_string(EthernetHeader::length_bytes);
        print_log(log);
        printf("Dropped UDP packet too short to contain Ethernet header\n");
    }else{
        // Set boundary of layer data. Data beyond boundary belongs to next upper layer afterwards.
        packet.set_boundary(OsiLayer::Link, EthernetHeader::length_bytes); // selecciona la trama Ethernet contenida en UDP?
        auto link_range = packet[OsiLayer::Link];
        EthernetHeader eth = decode_ethernet_header(link_range.begin(), link_range.end());
        if (callback_)
        {
            print_log("**callback_**");
            callback_(std::move(packet), eth);
        }
    }
}


//Cuando se ejecuta el callback tengo que comprobar si el buffer auxiliar contiene
void UartLink_rx::serial_handler(boost::system::error_code ec, std::size_t length){
    if (!ec){

        std::vector <TFrameInfo> frames = std::vector <TFrameInfo>();
        frames.clear();

        std::cout << "***************** Bytes received through serial port ********************** " << std::endl;
        std::cout <<"Number of bytes received: "<<length<<std::endl; 
        printf("Bytes received: ");
        mostrar(rx_buffer_.begin(), rx_buffer_.begin()+length); 

        //En el buffer completo introduzco la trama incompleta anterior (si la hay) y todos los datos recibidos
        comp_rx_buffer_.assign(aux_rx_buffer_.begin(), aux_rx_buffer_.end());  
        comp_rx_buffer_.insert(comp_rx_buffer_.end(), rx_buffer_.begin(), rx_buffer_.begin()+length);


        printf("Search buffer:\n");
        mostrar(comp_rx_buffer_.begin(), comp_rx_buffer_.end());  

        std::cout << "Searching for frames..." << std::endl; 
        contains(comp_rx_buffer_, SHORT_NAME, frames);  //devuelve un vector con el inicio al short name de todas las tramas recibidas

        if(!frames.empty()){
            size_t distancia;
            bool completa;

            for(int i = 0; i < frames.size(); i++){

                printf("------ FRAME ----- \n");
                print_log("Begin: " + *(frames.at(i).frame_inicio));
                printf("Frame size: %ld \n", frames.at(i).max_size);

                if(frames.at(i).defectuosa){
                    printf("Faulty frame! Discarted\n");
                }else{
                    if(i == (frames.size() -1)){ //última o primera (si solo hay una) -> las únicas que pueden tener problemas de completitud
                        printf("This is the last frame or the only one \n");
                        printf("Complete frame check starting...\n");
                        distancia = std::distance(frames.at(i).frame_inicio, comp_rx_buffer_.end());
                        print_log("Distance: " + std::to_string(distancia));
                        if(distancia < ADV_DATA -1){
                            //La trama no está completa 
                            //Almaceno lo recibido en un buffer auxiliar
                            printf("Incomplete frame (Reason 1)\n");
                            bytes_stored_ = std::distance(frames.at(i).frame_inicio, comp_rx_buffer_.end());
                            printf("Stored: %d bytes\n",bytes_stored_);
                            aux_rx_buffer_.insert(aux_rx_buffer_.end(), frames.at(i).short_name_inicio, comp_rx_buffer_.end()); 
                            completa = false;

                        }else if(frames.at(i).frame_inicio >= comp_rx_buffer_.end()){
                            //Ha recibido el short name pero no el inicio como tal de la trama ITS
                            printf("Incomplete frame (Reason 2)\n");
                            bytes_stored_ = 0;
                            printf("Stored: %d bytes\n",bytes_stored_);
                            aux_rx_buffer_.insert(aux_rx_buffer_.end(), frames.at(i).short_name_inicio, comp_rx_buffer_.end()); 
                            completa = false;

                        }else{
                            printf("Complete frame\n");
                            completa = true;
                        }

                    }else{ //resto
                        distancia = std::distance(frames.at(i).frame_inicio, frames.at(i+1).frame_inicio);
                        printf("Distance: %ld bytes\n", distancia);
                        completa = true;
                    }

                    if(completa){
                        printf("Frame process starting...\n");
                        aux_rx_buffer_.clear();
                        bytes_stored_ = 0;
                        process_frame(frames.at(i).frame_inicio, frames.at(i).frame_inicio+frames.at(i).max_size);
                    }
                }

                printf("------------------ \n");
            }

        }else{
            printf("No frames found. The bytes are stored.\n");
            bytes_stored_ += length;
            aux_rx_buffer_.insert(aux_rx_buffer_.end(), comp_rx_buffer_.begin(),comp_rx_buffer_.begin()+bytes_stored_);
        }

        rx_buffer_.assign(1024, 0x00);
        comp_rx_buffer_.clear();
        frames.clear();

        std::cout << "*************************************************************\n" << std::endl;

        serial_rx_.async_read_some(boost::asio::buffer(rx_buffer_), boost::bind(&UartLink_rx::serial_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
}

void UartLink_rx::do_receive()
{

    serial_rx_.async_read_some(boost::asio::buffer(rx_buffer_), boost::bind(&UartLink_rx::serial_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

}


void UartLink_rx::request(const access::DataRequest &request, std::unique_ptr<ChunkPacket> packet){}