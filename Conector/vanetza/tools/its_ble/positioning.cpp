#include "positioning.hpp"
#include <vanetza/common/stored_position_provider.hpp>

#ifdef SOCKTAP_WITH_GPSD
#   include "gps_position_provider.hpp"
#endif

using namespace vanetza;
namespace po = boost::program_options;

std::unique_ptr<vanetza::PositionProvider>
create_position_provider(boost::asio::io_service& io_service, const Runtime& runtime)
{
    std::unique_ptr<vanetza::PositionProvider> positioning;
    std::unique_ptr<StoredPositionProvider> stored { new StoredPositionProvider() };
    PositionFix fix;

    fix.timestamp = runtime.now();
    fix.latitude = 0 * units::degree; //double
    fix.longitude = 0 * units::degree;
    fix.confidence.semi_major = 0 * units::si::meter;
    fix.confidence.semi_minor = fix.confidence.semi_major;
    stored->position_fix(fix);
    positioning = std::move(stored);

    return positioning;
}

void add_positioning_options(po::options_description& options)
{
#ifdef SOCKTAP_WITH_GPSD
    const char* default_positioning = "gpsd";
#else
    const char* default_positioning = "static";
#endif

    options.add_options()
        ("positioning,p", po::value<std::string>()->default_value(default_positioning), "Select positioning provider")
#ifdef SOCKTAP_WITH_GPSD
        ("gpsd-host", po::value<std::string>()->default_value(gpsd::shared_memory), "gpsd's server hostname")
        ("gpsd-port", po::value<std::string>()->default_value(gpsd::default_port), "gpsd's listening port")
#endif
        ("latitude", po::value<double>()->default_value(48.7668616), "Latitude of static position")
        ("longitude", po::value<double>()->default_value(11.432068), "Longitude of static position")
        ("pos_confidence", po::value<double>()->default_value(5.0), "95% circular confidence of static position")
    ;
}
