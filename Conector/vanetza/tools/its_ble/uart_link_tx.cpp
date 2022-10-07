#include "uart_link_tx.hpp"
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

UartLink_tx::UartLink_tx(boost::asio::io_service &io_service, const std::string& dev_tx) : serial_stream_tx_(dev_tx)
{
    serial_stream_tx_.SetDTR(true); //true = high, false = low

    if(serial_stream_tx_.GetDTR()){
        print_log("DTR high");
    }else{
        print_log("DTR low");
    }
    serial_stream_tx_.SetBaudRate(BaudRate::BAUD_115200);
    serial_stream_tx_.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);
    serial_stream_tx_.SetParity(Parity::PARITY_DEFAULT); //NONE
    serial_stream_tx_.SetStopBits(StopBits::STOP_BITS_1);
    serial_stream_tx_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
}



void UartLink_tx::request(const access::DataRequest &request, std::unique_ptr<ChunkPacket> packet)
{
    print_log("**UartLink_tx::request**");

    packet->layer(OsiLayer::Link) = create_ethernet_header(request.destination_addr, request.source_addr, request.ether_type);

    std::array<boost::asio::const_buffer, layers_> const_buffers;
    for (auto &layer : osi_layer_range<OsiLayer::Link, OsiLayer::Application>())
    {
        const auto index = distance(OsiLayer::Link, layer);
        packet->layer(layer).convert(tx_buffers_[index]);
        const_buffers[index] = boost::asio::buffer(tx_buffers_[index]);
    }

    // Empleo tx_buffers_, un array de tamaño 6 formado por elementos de clase ByteBuffer. ByteByffer es un vector de uin8_t.
    std::cout << "Ethernet frame created" << std::endl;
    printf("Header size: %lu\n", tx_buffers_[0].size());
    printf("Payload size: %lu\n", tx_buffers_[1].size());
 
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < tx_buffers_[i].size(); j++)
        {
            printf("%02x", tx_buffers_[i].at(j));
        }
    }
    printf("\n");

    uint8_t tama = const_buffers[0].size() + const_buffers[1].size();
    serial_stream_tx_.write((char*) &tama, sizeof(uint8_t));
    serial_stream_tx_.write((const char*) const_buffers[0].data(), const_buffers[0].size());
    serial_stream_tx_.write((const char*) const_buffers[1].data(), const_buffers[1].size());

    std::cout << "********************************************************************************\n"
              << std::endl;
}

void UartLink_tx::indicate(IndicationCallback cb)
{    
    callback_ = cb;
}


