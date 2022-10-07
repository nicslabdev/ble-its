#include "ethernet_device.hpp"
#include "benchmark_application.hpp"
#include "cam_application.hpp"
#include "denm_application.hpp"
#include "cam_rx_application.hpp"
#include "hello_application.hpp"
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
        ("link-layer,l", po::value<std::string>()->default_value("ethernet"), "Link layer type")
        ("interface,i", po::value<std::string>()->default_value("lo"), "Network interface to use.")
        ("mac-address", po::value<std::string>(), "Override the network interface's MAC address.")
        ("require-gnss-fix", "Suppress transmissions while GNSS position fix is missing")
        ("gn-version", po::value<unsigned>()->default_value(1), "GeoNetworking protocol version to use.")
        ("cam-interval", po::value<unsigned>()->default_value(2000), "CAM sending interval in milliseconds.")
        ("print-rx-cam", "Print received CAMs")
        ("print-tx-cam", "Print generated CAMs")
        ("benchmark", "Enable benchmarking")
        ("applications,a", po::value<std::vector<std::string>>()->default_value({"ca"}, "ca")->multitoken(), "Run applications [ca, den, hello,benchmark, ca-rx]")
        ("non-strict", "Set MIB parameter ItsGnSnDecapResultHandling to NON_STRICT")
        //ItsGnSnDecapResultHandling -> STRICT supone descartar el paquete si el paquete no ha sido correctamente descrifrado y verificado
        //ItsGnSnDecapResultHandling -> NON_STRICT: el payload de GN-PDU pasa a la entidad de protocolo superior a través de la primitiva GN-DATA.Indication
        ("dev-tx", po::value<std::string>()->default_value("/dev/ttyACM0"), "Set the serial port device for reading data")
        ("dev-rx", po::value<std::string>()->default_value("/dev/ttyACM0"), "Set the serial port device for writig data")
        ("soft-ver", "Soft verification: generation time and certificate expiration unverified")
        
    ;
    add_positioning_options(options);
    add_security_options(options);

    po::positional_options_description positional_options;
    positional_options.add("interface", 1);

    po::variables_map vm;

    try {
        po::store(
            po::command_line_parser(argc, argv)
                .options(options)
                .positional(positional_options)
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
        /*
        Asio's io_service is the facilitator for operating on asynchronous 
        functions. Once an async operation is ready, it uses one of io_service's 
        running threads to call you back. If no such thread exists it uses its 
        own internal thread to call you.
        */
        TimeTrigger trigger(io_service);

        const char* device_name = vm["interface"].as<std::string>().c_str();
        //El constructor de EthernetDevice crea un socket local y se lo asigna al atributo local_socket_(::socket(AF_LOCAL, SOCK_DGRAM, 0))
        //AF_LOCAL para comunicar procesos en la misma máquina de forma eficiente
        //El constructor también asigna a interface_name_ el nombre de la interfaz que le pasamos (ej: eth0)
        EthernetDevice device(device_name); //device name puede ser eth0, lo
        vanetza::MacAddress mac_address = device.address();

        if (vm.count("mac-address")) {
            std::cout << "Using MAC address: " << vm["mac-address"].as<std::string>() << "." << std::endl;

            if (!parse_mac_address(vm["mac-address"].as<std::string>().c_str(), mac_address)) {
                std::cerr << "The specified MAC address is invalid." << std::endl;
                return 1;
            }
        }

        const std::string link_layer_name = vm["link-layer"].as<std::string>();
        const std::string serial_port_rx_dev = vm["dev-rx"].as<std::string>();
        const std::string serial_port_tx_dev = vm["dev-tx"].as<std::string>();
        auto link_layer =  create_link_layer(io_service, device, link_layer_name, serial_port_tx_dev, serial_port_rx_dev);
        //link_layer_name == ethernet  -> raw sockets
        //link_layer_name == udp
        //  1. se crea el endpoint (ip + puerto) multicast(ip::address::from_string("239.118.122.97"), 8947);
        //    2. hay dos sockets, uno para transmitir y otro para enviar. Se tx y rx tramas Ethernet
        //    3. Está permitido reutilizar la dirección (por eso dos sockets?)
        //    4. rx_socket_.set_option(ip::multicast::enable_loopback(false)) --> Socket option determining whether 
        //    outgoing multicast packets will be received on the same socket if it is a member of the multicast group.

        if (!link_layer) {
            std::cerr << "No link layer '" << link_layer_name << "' found." << std::endl;
            return 1;
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
        // TODO: make more MIB options configurable by command line flags
        gn::MIB mib;
        mib.vanetzaDisableBeaconing = true; //CUIDADO! True deshabilita los beacons
        mib.itsGnLocalGnAddr.mid(mac_address);
        mib.itsGnLocalGnAddr.is_manually_configured(true);
        mib.itsGnLocalAddrConfMethod = geonet::AddrConfMethod::Managed;
        /*The auto-address configuration method shall be used if the MIB attribute itsGnLocalAddrConfMethod is set to 
        AUTO (0). At start-up, the GeoAdhoc router shall assign its local GN_ADDR from the MIB attribute itsGnLocalGnAddr. 
        The local GN_ADDR shall not be changed unless the MIB attribute itsGnLocalAddrConfMethod is set to 
        MANAGED (1). */
        
        mib.itsGnSecurity = false; //indica si la seguridad de GN está activada o desactivada
        if (vm.count("non-strict")) {
            mib.itsGnSnDecapResultHandling = vanetza::geonet::SecurityDecapHandling::Non_Strict;
        }
        mib.itsGnProtocolVersion = vm["gn-version"].as<unsigned>();

        if (mib.itsGnProtocolVersion != 0 && mib.itsGnProtocolVersion != 1) {
            throw std::runtime_error("Unsupported GeoNetworking version, only version 0 and 1 are supported.");
        }

        if(vm.count("soft-ver")){
            printf("Soft-ver activado\n");
            mib.customSoftVerification = true;
        }else{
            printf("Soft-ver no activado\n");
            mib.customSoftVerification = false;
        }


        auto positioning = create_position_provider(io_service, vm, trigger.runtime());
        if (!positioning) {
            std::cerr << "Requested positioning method is not available\n";
            return 1;
        }

        auto security = create_security_entity(vm, trigger.runtime(), *positioning, mib.customSoftVerification);
        if (security) {
            mib.itsGnSecurity = true;
        }

        RouterContext context(mib, trigger, *positioning, security.get());
        context.require_position_fix(vm.count("require-gnss-fix") > 0);
        context.set_link_layer(link_layer.get());
        
        //std::map is a sorted associative container that contains key-value pairs with unique keys. Keys are sorted by using the comparison function Compare. 
        std::map<std::string, std::unique_ptr<Application>> apps;
        for (const std::string& app_name : vm["applications"].as<std::vector<std::string>>()) {
            if (apps.find(app_name) != apps.end()) {
                std::cerr << "application '" << app_name << "' requested multiple times, skip\n";
                continue;
            }
            if (app_name == "ca") {
                std::unique_ptr<CamApplication> ca {
                    new CamApplication(*positioning, trigger.runtime())
                };
                //Al crear la aplicación CAM se establece un callback que se va a llamar cuando expire un timer tras cam_interval_
                //Tras expirar ese timer se vuelve a establecer el callback para su posterior activación y se crea el mensaje CAM

                ca->set_interval(std::chrono::milliseconds(vm["cam-interval"].as<unsigned>()));
                ca->print_received_message(vm.count("print-rx-cam") > 0);
                ca->print_generated_message(vm.count("print-tx-cam") > 0);
                //emplace() Inserts a new element into the container constructed in-place with the given args if there is no element with the key in the container.
                apps.emplace(app_name, std::move(ca));
            } else if(app_name == "den"){
                std::unique_ptr<DenmApplication> den {
                    new DenmApplication(*positioning, trigger.runtime())
                };

                den->set_interval(std::chrono::milliseconds(vm["cam-interval"].as<unsigned>()));
                den->print_received_message(vm.count("print-rx-cam") > 0);
                den->print_generated_message(vm.count("print-tx-cam") > 0);
                //emplace() Inserts a new element into the container constructed in-place with the given args if there is no element with the key in the container.
                apps.emplace(app_name, std::move(den));
            }else if(app_name == "ca-rx"){
                std::unique_ptr<CamRxApplication> p {
                    new CamRxApplication()
                };

                p->print_received_message(vm.count("print-rx-cam") > 0);
                apps.emplace(app_name, std::move(p));

            }else if (app_name == "hello") {
                std::unique_ptr<HelloApplication> hello {
                    new HelloApplication(io_service, std::chrono::milliseconds(800))
                };
                apps.emplace(app_name, std::move(hello));
            } else if (app_name == "benchmark") {
                std::unique_ptr<BenchmarkApplication> benchmark {
                    new BenchmarkApplication(io_service)
                };
                apps.emplace(app_name, std::move(benchmark));
            } else {
                std::cerr << "skip unknown application '" << app_name << "'\n";
            }
        }

        if (apps.empty()) {
            std::cerr << "Warning: No applications are configured, only GN beacons will be exchanged\n";
        }

        for (const auto& app : apps) {
            std::cout << "Enable application '" << app.first << "'...\n";
            context.enable(app.second.get());
        }

        io_service.run(); //se comienza a ejecutar el bucle de procesamiento de eventos
        
    } catch (PositioningException& e) {
        std::cerr << "Exit because of positioning error: " << e.what() << std::endl;
        return 1;
    } catch (std::exception& e) {
        std::cerr << "Exit: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
