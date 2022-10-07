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

class UartLink_tx : public LinkLayer
{
public:
    UartLink_tx(boost::asio::io_service&, const std::string& dev_tx);

    void indicate(IndicationCallback) override;
    void request(const vanetza::access::DataRequest&, std::unique_ptr<vanetza::ChunkPacket>) override;


private:

    static constexpr std::size_t layers_ = num_osi_layers(vanetza::OsiLayer::Link, vanetza::OsiLayer::Application);
    IndicationCallback callback_;
    std::array<vanetza::ByteBuffer, layers_> tx_buffers_;
    SerialStream serial_stream_tx_;

};

