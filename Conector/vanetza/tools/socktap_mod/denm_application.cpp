#include "denm_application.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/denm.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
//#include <vanetza/facilities/denm_functions.hpp>
#include <chrono>
#include <exception>
#include <functional>
#include <iostream>

// This is a very simple CA application sending CAMs at a fixed rate.

using namespace vanetza;
using namespace std::chrono;

DenmApplication::DenmApplication(PositionProvider& positioning, Runtime& rt) :

    positioning_(positioning), runtime_(rt), denm_interval_(seconds(1))
{
    schedule_timer();
}

void DenmApplication::set_interval(Clock::duration interval) //en caso de que se modifique con las opciones a la hora de ejecutar socktap
{
    denm_interval_ = interval; //DA ERROR
    runtime_.cancel(this);
    schedule_timer();
}

void DenmApplication::print_generated_message(bool flag) //depende de la opción --print-tx-ca
{
    print_tx_msg_ = flag;
}

void DenmApplication::print_received_message(bool flag) //depende de la opción --print-rx-ca
{
    print_rx_msg_ = flag;
}

DenmApplication::PortType DenmApplication::port()
{
    return btp::ports::DENM; //puerto BTP
}

void DenmApplication::indicate(const DataIndication& indication, UpPacketPtr packet)
{
    asn1::PacketVisitor<asn1::Denm> visitor;
    std::shared_ptr<const asn1::Denm> denm = boost::apply_visitor(visitor, *packet);

    std::cout << "Denm application received a packet with " << (denm ? "decodable" : "broken") << " content" << std::endl;
    if (denm && print_rx_msg_) {
        std::cout << "Received DENM contains\n";
        //print_indented(std::cout, *cam, "  ", 1);
    }
}

void DenmApplication::schedule_timer()
{
    runtime_.schedule(denm_interval_, std::bind(&DenmApplication::on_timer, this, std::placeholders::_1), this);
}

void DenmApplication::on_timer(Clock::time_point)
{
    schedule_timer(); //se vuelve a establecer el callback para ir generando mensajes CAM cada cam_interval_
    vanetza::asn1::Denm message;

    ItsPduHeader_t& header = message->header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_denm; //cada mensaje (cam, denm, ivi) tiene un identificador
    header.stationID = 1; // some dummy value

    DecentralizedEnvironmentalNotificationMessage_t& denm = message->denm;
    
    //Management Container (obligatorio) - información sobre gestión y el protocolo
    ManagementContainer_t& manage = denm.management;

    manage.actionID.originatingStationID = 0;
    manage.actionID.sequenceNumber = 0;
    
    const auto time_now = duration_cast<milliseconds>(runtime_.now().time_since_epoch());
    uint64_t time = time_now.count();
	TimestampIts_t* timestamp = (TimestampIts_t*)(malloc(sizeof(TimestampIts_t)));
	timestamp->buf = (uint8_t*) malloc(8); // timestamp needs 42 bits in standard
	timestamp->size = 8;
	memcpy(timestamp->buf, &time, 8);
	denm.management.detectionTime = *timestamp;
	denm.management.referenceTime = *timestamp;

	denm.management.stationType = StationType_passengerCar;
	
    auto position = positioning_.position_fix();
    if (!position.confidence) {
        std::cerr << "Skipping DENM, because no good position is available, yet." << std::endl;
        return;
    }
    //copy(position, denm.management.eventPosition);

	denm.management.validityDuration = static_cast<long*>(calloc(sizeof(long), 1));
	denm.management.validityDuration = 0;

    /*denm->denm.management.termination = static_cast<long*>(calloc(sizeof(long), 1));
    *denm->denm.management.termination = Termination_isCancellation;

    denm->denm.management.transmissionInterval = static_cast<long*>(calloc(sizeof(long), 1));
    *denm->denm.management.transmissionInterval = TransmissionInterval_tenSeconds;

    denm->denm.management.relevanceDistance =  static_cast<long*>(calloc(sizeof(long), 1));
    *denm->denm.management.relevanceDistance =  RelevanceDistance_lessThan10km;

    denm->denm.management.relevanceTrafficDirection =  static_cast<long*>(calloc(sizeof(long), 1));
    *denm->denm.management.relevanceTrafficDirection =  RelevanceTrafficDirection_allTrafficDirections;

    //Situation Container -> information that describes the detected event
    denm.situation = static_cast<SituationContainer_t*>(calloc(sizeof(SituationContainer_t), 1));
    
    *denm.situation->informationQuality = InformationQuality_unavailable;

    *denm.situation->eventType.causeCode = CauseCodeType_trafficCondition;
    *denm.situation->eventType.subCauseCode = 0;

    *denm.situation->linkedCause = static_cast<CauseCode_t*>(calloc(sizeof(CauseCode_t), 1));
    *denm.situation->linkedCause->causeCode = CauseCodeType_aquaplannning;
    *denm.situation->linkedCause->subCauseCode = 0;
    
    //Lista de puntos en los que se ha detectado 
    //*denm.situation->eventHistory = static_cast<EventHistory_t*>(calloc(sizeof(EventHistory_t), 1));
    
    //LOCATION CONTAINER - 
    denm.location= static_cast<LocationContainer_t*>(calloc(sizeof(LocationContainer_t), 1));
    
    *denm.location->eventSpeed = static_cast<Speed_t*>(calloc(sizeof(Speed_t), 1));
    *denm.location->eventSpeed->speedValue = SpeedValue_standstill;
    *denm.location->eventSpeed->speedConfidence = SpeedConfidence_unavailable;

    *denm.location->eventPositionHeading = static_cast<Heading_t*>(calloc(sizeof(Heading_t), 1));
    *denm.location->eventPositionHeading->headingConfidence = HeadingConfidence_unavailable;
    *denm.location->eventPositionHeading->headingValue = HeadingValue_unavailable;

    *denm.location->roadType = static_cast<long*>(calloc(sizeof(long), 1));
    *denm.location->roadType = RoadType_nonUrban_WithStructuralSeparationToOppositeLanes;
    
    //*denm.location->traces.


    //A LA CARTE CONTAINER - additional information that is not provided by other containers
    denm.alacarte = static_cast<Alacarteontainer_t*>(calloc(sizeof(AlacarteContainer_t), 1));
    
    *denm.alacarte->lanePosition = static_cast<LanePosition_t*>(calloc(sizeof(LanePosition_t), 1));
    *denm.alacarte->lanePosition = LanePosition_offTheRoad;

    *denm.alacarte->impactReduction = static_cast<ImpactReductionContainer_t*>(calloc(sizeof(ImpactReductionContainer_t), 1));
    *denm.alacarte->impactReduction->heightLonCarrLeft = HeightLonCarr_oneCentimeter;
    *denm.alacarte->impactReduction->heightLonCarrRight = HeightLonCarr_oneCentimeter;
    *denm.alacarte->impactReduction->posLonCarrLeft = PosLonCarr_oneCentimeter;
    *denm.alacarte->impactReduction->posLonCarrRight = PosLonCarr_oneCentimeter;
    *denm.alacarte->impactReduction->posCentMass = PosCentMass_tenCentimeters;
    *denm.alacarte->impactReduction->wheelBaseVehicle = WheelBaseVehicle_tenCentimeters;
    *denm.alacarte->impactReduction->turningRadius = TurningRadius_point4Meters;
    *denm.alacarte->impactReduction->posFrontAx = PosFrontAx_tenCentimeters;
    
    *denm.alacarte->impactReduction->positionOfOccupants.buf = static_cast<uint8_t*>(calloc(sizeof(uint8_t), 20));
    *denm.alacarte->impactReduction->positionOfOccupants.buf = PositionOfOccupants_row1RightOccupied;
    *denm.alacarte->impactReduction->positionOfOccupants.size = 20;
    *denm.alacarte->impactReduction->positionOfOccupants.bits_unused = 0;

    //*denm.alacarte->impactReduction->positionOfPillars = 


*/



    const auto t_now = duration_cast<milliseconds>(runtime_.now().time_since_epoch());
    int ret1 = asn_long2INTEGER(&message->denm.management.detectionTime, t_now.count());
    int ret2 = asn_long2INTEGER(&message->denm.management.referenceTime, TimestampIts_utcStartOf2004);
    std::cout << "ret 1 : " << ret1 << "ret 2 : " << ret2 << std::endl;
    assert(ret1 + ret2 == 0);

    std::string error;
    if (!message.validate(error)) {
        throw std::runtime_error("Error" + error);
    }
    
    std::cout << "Generated DENM\n";

    DownPacketPtr packet { new DownPacket() }; //crea un paquete que permite que las capas vayan añadiendo bytes sin preocuparse del resto de capas
    packet->layer(OsiLayer::Application) = std::move(message);//introduzco el mensaje CAM en la capa de aplicación


    DataRequest request; 
    request.its_aid = aid::DEN;

    request.transport_type = geonet::TransportType::SHB; 
    request.communication_profile = geonet::CommunicationProfile::Unspecified; 

    auto confirm = Application::request(request, std::move(packet));

    if (!confirm.accepted()) {
        throw std::runtime_error("DENM application data request failed");
    }
}
