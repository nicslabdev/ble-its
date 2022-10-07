#include "cam_rx_application.hpp"
#include "cam_tx_application.hpp"
#include "link_layer.hpp"
#include "positioning.hpp"
#include "router_context.hpp"
#include "security.hpp"
#include "time_trigger.hpp"
#include <boost/asio/serial_port.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

namespace asio = boost::asio;
namespace gn = vanetza::geonet;
namespace po = boost::program_options;
using namespace vanetza;

int main(int argc, const char** argv)
{

    po::options_description options("Allowed options");
    options.add_options()
        ("help", "Print out available options.")
        ("mac-address", po::value<std::string>()->default_value("00:00:00:00:00:01"), "Override the network interface's MAC address.")
        ("cam-interval", po::value<unsigned>()->default_value(2000), "CAM sending interval in milliseconds.")
        ("print-rx-cam", "Print received CAMs")
        ("print-tx-cam", "Print generated CAMs")
        ("non-strict", "Set MIB parameter ItsGnSnDecapResultHandling to NON_STRICT")
        ("rol", po::value<std::string>()->default_value("tx"), "Set the rol of the device (tx or rx)")
        ("dev", po::value<std::string>()->default_value("/dev/ttyACM0"), "Set the serial port device(/dev/ttyACM0)")

    ;
    add_security_options(options);

    po::variables_map vm;

    try {
        po::store(
            po::command_line_parser(argc, argv)
                .options(options)
                .run(),
            vm
        );
        po::notify(vm);
    } catch (po::error& e) {
        std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
        std::cerr << options << std::endl;
        return 1;
    }

    if (vm.count("help")) {
        std::cout << options << std::endl;
        return 1;
    }


    try {
        asio::io_service io_service;
        vanetza::MacAddress mac_address;
        TimeTrigger trigger(io_service);


        if(!vm.count("dev")){
            std::cerr << "A serial port is required" << std::endl;
            return 1;
        }

        std::cout << "Device rol: " << vm["rol"].as<std::string>() <<std::endl;
        
        if (vm.count("mac-address")) {
            std::cout << "Using MAC address: " << vm["mac-address"].as<std::string>() << "." << std::endl;

            if (!parse_mac_address(vm["mac-address"].as<std::string>().c_str(), mac_address)) {
                std::cerr << "The specified MAC address is invalid." << std::endl;
                return 1;
            }
        }else{
            std::cerr << "The MAC address is required" << std::endl;
            return 1;
        }

        const std::string serial_port = vm["dev"].as<std::string>();
        const std::string rol = vm["rol"].as<std::string>();
        
        std::unique_ptr<LinkLayer> link_layer;

        if(rol == "rx"){
            printf("Creating link layer...\n");
            link_layer =  create_link_layer_rx(io_service, serial_port);
            if (link_layer == nullptr) {
                std::cerr << "No link layer found." << std::endl;
                return 1;
            }            

        }else if(rol == "tx"){
            printf("Creating link layer...\n");
            link_layer =  create_link_layer_tx(io_service, serial_port);
            if (link_layer == nullptr) {
                std::cerr << "No link layer found." << std::endl;
                return 1;
            }
        }

        auto signal_handler = [&io_service](const boost::system::error_code& ec, int signal_number) {
            if (!ec) {
                std::cout << "Termination requested." << std::endl;
                io_service.stop();
            }
        };

        asio::signal_set signals(io_service, SIGINT, SIGTERM);
        signals.async_wait(signal_handler);

        // configure management information base
        gn::MIB mib;
        mib.vanetzaDisableBeaconing = true; //CUIDADO! True deshabilita los beacons
        mib.itsGnLocalGnAddr.mid(mac_address);
        mib.itsGnLocalGnAddr.is_manually_configured(true);
        mib.itsGnLocalAddrConfMethod = geonet::AddrConfMethod::Managed; //no se que poner aqui
        mib.itsGnSecurity = false; //indica si la seguridad de GN está activada o desactivada
        if (vm.count("non-strict")) {
            mib.itsGnSnDecapResultHandling = vanetza::geonet::SecurityDecapHandling::Non_Strict;
        }
        mib.itsGnProtocolVersion = 1;

        if (mib.itsGnProtocolVersion != 0 && mib.itsGnProtocolVersion != 1) {
            throw std::runtime_error("Unsupported GeoNetworking version, only version 0 and 1 are supported.");
        }

        if(vm.count("soft-ver")){
            print_log("Soft verification enabled");
            mib.customSoftVerification = true;
        }else{
            print_log("Soft verification disabled");
            mib.customSoftVerification = false;
        }


        auto positioning = create_position_provider(io_service, trigger.runtime());
        if (!positioning) { //terminar de modificar la función
            std::cerr << "Requested positioning method is not available\n";
            return 1;
        }
        
        auto security = create_security_entity(vm, trigger.runtime(), *positioning, mib.customSoftVerification);
        if (security) {
            mib.itsGnSecurity = true;
        }
        
        RouterContext context(mib, trigger, *positioning, security.get());
        context.require_position_fix(false);
        context.set_link_layer(link_layer.get());
            

        std::map<std::string, std::unique_ptr<Application>> apps;
    
        if(rol == "rx"){
            std::unique_ptr<Cam_rx_application> ca_rx {
                new Cam_rx_application(*positioning, trigger.runtime())
            };
            ca_rx->print_received_message(vm.count("print-rx-cam") > 0);
            apps.emplace("rx", std::move(ca_rx));

        }else if (rol == "tx"){
            std::unique_ptr<Cam_tx_application> ca_tx {
                new Cam_tx_application(*positioning, trigger.runtime())
            };
            ca_tx->set_interval(std::chrono::milliseconds(vm["cam-interval"].as<unsigned>()));
            ca_tx->print_generated_message(vm.count("print-tx-cam") > 0);
            apps.emplace("tx", std::move(ca_tx));
        } 

        if (apps.empty()) {
            std::cerr << "Warning: No applications are configured\n";
        }

        for (const auto& app : apps) {
            std::cout << "Enable application '" << app.first << "'...\n";
            context.enable(app.second.get());
        }
               
        io_service.run(); //se comienza a ejecutar el bucle de procesamiento de eventos
        
    } catch (std::exception& e) {
        std::cerr << "Exit: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
