#include "application.hpp"
#include <vanetza/common/clock.hpp>
#include <vanetza/common/position_provider.hpp>
#include <vanetza/common/runtime.hpp>
#include <vanetza/asn1/its/LanePosition.h>

class Cam_tx_application : public Application
{
public:
    Cam_tx_application(vanetza::PositionProvider& positioning, vanetza::Runtime& rt);
    PortType port() override;
    void set_interval(vanetza::Clock::duration);
    void print_generated_message(bool flag);
    void indicate(const DataIndication&, UpPacketPtr) override;


private:
    void schedule_timer();
    void on_timer(vanetza::Clock::time_point);


    vanetza::PositionProvider& positioning_;
    vanetza::Runtime& runtime_;
    vanetza::Clock::duration cam_interval_;
    bool print_tx_msg_ = false;

};

