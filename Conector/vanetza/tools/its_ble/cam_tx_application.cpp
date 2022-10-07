#include "cam_tx_application.hpp"
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
#include "log.hpp"

// This is a very simple CA application sending CAMs at a fixed rate.

using namespace vanetza;
using namespace vanetza::facilities;
using namespace std::chrono;
using namespace log_module;

Cam_tx_application::Cam_tx_application(PositionProvider& positioning, Runtime& rt) :

    positioning_(positioning), runtime_(rt), cam_interval_(seconds(1))
{
    schedule_timer();
}

void Cam_tx_application::set_interval(Clock::duration interval) //en caso de que se modifique con las opciones a la hora de ejecutar socktap
{
    cam_interval_ = interval;
    runtime_.cancel(this);
    schedule_timer();
}

void Cam_tx_application::print_generated_message(bool flag) //depende de la opción --print-tx-ca
{
    print_tx_msg_ = flag;
}

void Cam_tx_application::indicate(const DataIndication& indication, UpPacketPtr packet){}


Cam_tx_application::PortType Cam_tx_application::port()
{
    return btp::ports::CAM; //puerto BTP
}

void Cam_tx_application::schedule_timer()
{
    runtime_.schedule(cam_interval_, std::bind(&Cam_tx_application::on_timer, this, std::placeholders::_1), this);
}

void Cam_tx_application::on_timer(Clock::time_point)
{
    schedule_timer(); //se vuelve a establecer el callback para ir generando mensajes CAM cada cam_interval_
    vanetza::asn1::Cam message;

    ItsPduHeader_t& header = message->header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_cam; //cada mensaje (cam, denm, ivi) tiene un identificador
    header.stationID = 1; // some dummy value

    const auto time_now = duration_cast<milliseconds>(runtime_.now().time_since_epoch());
    uint16_t gen_delta_time = time_now.count();

    CoopAwareness_t& cam = message->cam;
    cam.generationDeltaTime = gen_delta_time * GenerationDeltaTime_oneMilliSec;
    //cam.generationDeltaTime es el tiempo de generación del mensaje CAM

    auto position = positioning_.position_fix();

    if (!position.confidence) {
        std::cerr << "Skipping CAM, because no good position is available, yet." << std::endl;
        return;
    }

    //cam.Parameters es la secuencia de contenedores CAM tanto obligatorios como opcionales
    //Basic Container -> contenedor obligatorio de CAM. Contiene el tipo de la estación origen y la posición de la estación
    BasicContainer_t& basic = cam.camParameters.basicContainer;
    basic.stationType = StationType_passengerCar;
    copy(position, basic.referencePosition);

    //HighFrequencyContainer puede ser de dos tipos (CHOICE en ASN1): BasicVehicleContainerHighFrequency o rsuContainerHighFrequency
    cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    //BasicVehicleContainerHighFrequency es un contenedor de alta frecuencia obligatorio cuando la estación ITS origen es un vehículo
    BasicVehicleContainerHighFrequency& bvc = cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
    bvc.heading.headingValue = 0; //wgs84North(0), wgs84East(900), wgs84South(1800), wgs84West(2700), unavailable(3601)
    bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;

    bvc.speed.speedValue = 0;
    bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec;

    bvc.driveDirection = DriveDirection_forward;
    bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;

    bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    bvc.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_noTrailerPresent;
    bvc.vehicleWidth = VehicleWidth_unavailable;

    bvc.curvature.curvatureValue = 0;
    bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;

    bvc.yawRate.yawRateValue = YawRateValue_unavailable;

    AccelerationControl_t	*accelerationControl = vanetza::asn1::allocate<AccelerationControl_t>();
    uint8_t *bitstring_buf = (uint8_t *)malloc(sizeof(uint8_t));
    *bitstring_buf = AccelerationControl_accEngaged;
    accelerationControl->buf = bitstring_buf;
    accelerationControl->size = 1;
    accelerationControl->bits_unused = 1;
    bvc.accelerationControl = accelerationControl;

    LanePosition_t	*lanePosition = vanetza::asn1::allocate<LanePosition_t>();
    *lanePosition = LanePosition_offTheRoad;
    bvc.lanePosition = lanePosition;

    SteeringWheelAngle_t *steering_wheel = vanetza::asn1::allocate<SteeringWheelAngle_t>();
    steering_wheel->steeringWheelAngleConfidence = SteeringWheelAngleConfidence_unavailable;
    steering_wheel->steeringWheelAngleValue = SteeringWheelAngleValue_unavailable;
    bvc.steeringWheelAngle = steering_wheel;

    LateralAcceleration_t *lateral_acc = vanetza::asn1::allocate<LateralAcceleration_t>();
    lateral_acc->lateralAccelerationConfidence = AccelerationConfidence_unavailable;
    lateral_acc->lateralAccelerationValue = LateralAccelerationValue_unavailable;
    bvc.lateralAcceleration = lateral_acc;

    VerticalAcceleration_t *vertical_acc = vanetza::asn1::allocate<VerticalAcceleration_t>();
    vertical_acc->verticalAccelerationConfidence = AccelerationConfidence_unavailable;
    vertical_acc->verticalAccelerationValue = VerticalAccelerationValue_unavailable;
    bvc.verticalAcceleration = vertical_acc;

    PerformanceClass_t *performance = vanetza::asn1::allocate<PerformanceClass_t>();
    *performance = PerformanceClass_unavailable;
    bvc.performanceClass = performance;

    CenDsrcTollingZoneID_t *cendsrctol_id = vanetza::asn1::allocate<CenDsrcTollingZoneID_t>();
    *cendsrctol_id = 0;
    CenDsrcTollingZone_t *cendsrctol = vanetza::asn1::allocate<CenDsrcTollingZone_t>();
    cendsrctol->cenDsrcTollingZoneID = cendsrctol_id;
    cendsrctol->protectedZoneLatitude = Latitude_unavailable;
    cendsrctol->protectedZoneLongitude = Longitude_unavailable;
    bvc.cenDsrcTollingZone = cendsrctol;

    //Low Frequency Container
    /*LowFrequencyContainer *low_f_containter = vanetza::asn1::allocate<LowFrequencyContainer>();
    low_f_containter->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
    low_f_containter->choice.basicVehicleContainerLowFrequency.vehicleRole = VehicleRole_default;
    
    uint8_t *bitstring_buf2 = (uint8_t *)malloc(sizeof(uint8_t));
    *bitstring_buf2 = ExteriorLights_lowBeamHeadlightsOn;
    low_f_containter->choice.basicVehicleContainerLowFrequency.exteriorLights.buf = bitstring_buf2;
    low_f_containter->choice.basicVehicleContainerLowFrequency.exteriorLights.size = 1;
    low_f_containter->choice.basicVehicleContainerLowFrequency.exteriorLights.bits_unused = 0;
    
    PathHistory_t *ph = vanetza::asn1::allocate<PathHistory_t>(); 
    
    struct PathPoint *p0 = vanetza::asn1::allocate<PathPoint>(); 
    PathDeltaTime_t *path_dt = vanetza::asn1::allocate<PathDeltaTime_t>();
    *path_dt = PathDeltaTime_tenMilliSecondsInPast;
    DeltaReferencePosition_t *drp = vanetza::asn1::allocate<DeltaReferencePosition_t>();
    drp->deltaAltitude = DeltaAltitude_unavailable;
    drp->deltaLatitude = DeltaLatitude_unavailable;
    drp->deltaLongitude = DeltaLongitude_unavailable;    
    p0->pathDeltaTime = path_dt;
    p0->pathPosition = *drp;

    ASN_SEQUENCE_ADD(ph, p0);

    struct PathPoint *p1 = vanetza::asn1::allocate<PathPoint>(); 
    PathDeltaTime_t *path_dt1 = vanetza::asn1::allocate<PathDeltaTime_t>();
    *path_dt1 = PathDeltaTime_tenMilliSecondsInPast;
    DeltaReferencePosition_t *drp1 = vanetza::asn1::allocate<DeltaReferencePosition_t>();
    drp1->deltaAltitude = DeltaAltitude_unavailable;
    drp1->deltaLatitude = DeltaLatitude_unavailable;
    drp1->deltaLongitude = DeltaLongitude_unavailable;    
    p1->pathDeltaTime = path_dt1;
    p1->pathPosition = *drp1;

    ASN_SEQUENCE_ADD(ph, p1);

    low_f_containter->choice.basicVehicleContainerLowFrequency.pathHistory = *ph;
    cam.camParameters.lowFrequencyContainer = low_f_containter;*/

    std::string error;
    if (!message.validate(error)) {
        throw std::runtime_error("Invalid high frequency CAM: %s" + error);
    }

    printf("********************************************************************************\n");
    printf("Creating ITS frame...\n");


    if (print_tx_msg_) { //si está activo este flag muestra por pantalla el mensaje generado
        printf("Generated CAM contains:\n");
        print_indented_mod(std::cout, message, "  ", 1);
    }

  
    DownPacketPtr packet { new DownPacket() }; //crea un paquete que permite que las capas vayan añadiendo bytes sin preocuparse del resto de capas
    packet->layer(OsiLayer::Application) = std::move(message);//introduzco el mensaje CAM en la capa de aplicación


    //ETSI TS 102 636-5-1 V1.1.1 (2011-02) (pag 12)
    DataRequest request; //vanetza::btp::DataRequest
    //The ITS-Application Identifier (ITS-AID) as given in ETSI TR 102 965 [i.18] indicates the overall type of permissions being granted: for example, there is an ITS-AID that indicates that the sender is entitled to send CAMs. (seguridad)
    request.its_aid = aid::CA; 
    /*A CAM may rely on the services provided by the GeoNetworking/BTP stack. If this stack is used, the GN packet 
    transport type Single-Hop Broadcasting (SHB) shall be used. In this scenario only nodes in direct communication range 
    may receive the CAM. */
    request.transport_type = geonet::TransportType::SHB; //The GN Packet transport type parameter specifies the packet transport type (GeoUnicast, SHB, TSB, GeoBroadcast, GeoAnycast). 
    request.communication_profile = geonet::CommunicationProfile::Unspecified; //Unspecified, ITS-G5, or LTE-V2X 
    //The GN Communication profile parameter determines the LL protocol entity (unspecified, ITS-G5A).


    auto confirm = Application::request(request, std::move(packet));
    //confirm es un objeto de la clase vanetza::geonet::DataConfirm
    //se asume que se va a utilizar la cabecera BTP-B de vanetza
    //Dependiendo del tipo de transporte utilizado (SHB o GBC) se va a hacer un request diferente. Los dos tipos de request
    //coinciden en que la información pertinente se va a almacenar en un DataRequest de la capa de red (geoNetworking)
    //En caso de transporte gbc se va a almacenar también el destino, de la clase Area
    //En caso de transporte shb no se almacena esta información
    //Una función devuelve un GbcDataRequest, y otra un ShbDataRequest

    //Desde la clase Application se realiza el DataRequest de aplicación -> Transporte y de Transporte -> Red
    if (!confirm.accepted()) {
        throw std::runtime_error("CAM application data request failed");
    }
}