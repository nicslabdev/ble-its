#ifndef UDP_LINK_HPP_A16QFBX3
#define UDP_LINK_HPP_A16QFBX3

#include "link_layer.hpp"
#include <vanetza/common/byte_buffer.hpp>
#include <boost/asio/ip/udp.hpp>
#include <vanetza/common/byte_buffer.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <array>
#include <vector>
#include <libserial/SerialStream.h>
#include <libserial/SerialPort.h>

using namespace vanetza;
using namespace LibSerial;

class UdpLink : public LinkLayer
{
public:
    UdpLink(boost::asio::io_service&, const boost::asio::ip::udp::endpoint&, const std::string& dev_tx, const std::string& dev_rx);

    void indicate(IndicationCallback) override;
    void request(const vanetza::access::DataRequest&, std::unique_ptr<vanetza::ChunkPacket>) override;


private:
    void do_receive();
    void serial_handler(boost::system::error_code ec, std::size_t length);
    void process_frame(ByteBuffer::iterator inicio, ByteBuffer::iterator final);

    static constexpr std::size_t layers_ = num_osi_layers(vanetza::OsiLayer::Link, vanetza::OsiLayer::Application);

    boost::asio::ip::udp::endpoint multicast_endpoint_;
    boost::asio::ip::udp::socket tx_socket_;
    boost::asio::ip::udp::socket rx_socket_;
    std::array<vanetza::ByteBuffer, layers_> tx_buffers_;
    vanetza::ByteBuffer rx_buffer_;
    vanetza::ByteBuffer comp_rx_buffer_; //buffer que va a contener la trama 
    vanetza::ByteBuffer aux_rx_buffer_; //buffer auxiliar. Va a almacenar la trama incompleta, por eso no es necesario que tenga un tamaño mayor a 251
    uint8_t bytes_stored_; //bytes que se han almacenado
    boost::asio::ip::udp::endpoint rx_endpoint_;
    IndicationCallback callback_;

    boost::asio::posix::stream_descriptor input_stream_;
    boost::asio::serial_port serial_rx_;
    SerialStream serial_stream_tx_;

};

#endif /* UDP_LINK_HPP_A16QFBX3 */

