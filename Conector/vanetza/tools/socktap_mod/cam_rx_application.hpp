#ifndef CAM_RX_APPLICATION_HPP
#define CAM_RX_APPLICATION_HPP

#include "application.hpp"
#include <vanetza/common/clock.hpp>
#include <vanetza/common/position_provider.hpp>
#include <vanetza/common/runtime.hpp>
#include <boost/thread/mutex.hpp>



class CamRxApplication : public Application
{
public:
    CamRxApplication();
    PortType port() override;
    void indicate(const DataIndication&, UpPacketPtr) override;
    void print_received_message(bool flag);

private:

    bool print_rx_msg_ = false;

};

#endif /* CAM_RX_APPLICATION */
