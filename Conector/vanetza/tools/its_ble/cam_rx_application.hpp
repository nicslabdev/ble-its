
#include "application.hpp"
#include <vanetza/common/clock.hpp>
#include <vanetza/common/position_provider.hpp>
#include <vanetza/common/runtime.hpp>


class Cam_rx_application : public Application
{
public:
    Cam_rx_application(vanetza::PositionProvider& positioning, vanetza::Runtime& rt);
    PortType port() override;
    void indicate(const DataIndication&, UpPacketPtr) override;
    void print_received_message(bool flag);

private:
    vanetza::PositionProvider& positioning_;
    vanetza::Runtime& runtime_;
    bool print_rx_msg_ = false;

};

