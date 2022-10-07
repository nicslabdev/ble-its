#include "cam_rx_application.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <chrono>
#include <exception>
#include <functional>
#include <iostream>
#include <boost/thread/mutex.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/program_options.hpp>

using namespace vanetza;
using namespace vanetza::facilities;
using namespace std::chrono;
using namespace boost::program_options;


CamRxApplication::CamRxApplication(){}

void CamRxApplication::print_received_message(bool flag) //depende de la opción --print-rx-ca
{
    print_rx_msg_ = flag;
}


Application::PortType CamRxApplication::port()
{
    return btp::ports::CAM; //puerto BTP
}

void CamRxApplication::indicate(const DataIndication& indication, UpPacketPtr packet)
{
    printf("**CamRxApplication::indicate**\n\n");
    asn1::PacketVisitor<asn1::Cam> visitor;
    std::shared_ptr<const asn1::Cam> cam = boost::apply_visitor(visitor, *packet);

    std::cout << "CAM_RX application received a packet with " << (cam ? "decodable" : "broken") << " content" << std::endl;
    if (cam && print_rx_msg_) {
        std::cout << "Received CAM contains\n";
        print_indented(std::cout, *cam, "  ", 1);
    }
}