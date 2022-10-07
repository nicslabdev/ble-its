#ifndef LINK_LAYER_HPP_FGEK0QTH
#define LINK_LAYER_HPP_FGEK0QTH

#include "ethernet_device.hpp"
#include <vanetza/access/interface.hpp>
#include <vanetza/net/cohesive_packet.hpp>
#include <vanetza/net/ethernet_header.hpp>
#include <memory>
#include <string>
#include <boost/asio/read.hpp>
#include <boost/asio/buffers_iterator.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>

using namespace vanetza;

class LinkLayerIndication
{
public:
    using IndicationCallback = std::function<void(vanetza::CohesivePacket&&, const vanetza::EthernetHeader&)>;

    virtual void indicate(IndicationCallback) = 0;
    virtual ~LinkLayerIndication() = default;
};

class LinkLayer : public vanetza::access::Interface, public LinkLayerIndication
{
public:
    void mostrar(ByteBuffer::iterator inicio, ByteBuffer::iterator final);
    void setDTR(boost::asio::serial_port &serial);
    void set(boost::asio::serial_port &serial);
    int get(boost::asio::serial_port &serial);
};

std::unique_ptr<LinkLayer>
create_link_layer_tx(boost::asio::io_service&, const std::string& serial_port_tx);

std::unique_ptr<LinkLayer>
create_link_layer_rx(boost::asio::io_service&, const std::string& serial_port_rx);




#endif /* LINK_LAYER_HPP_FGEK0QTH */

