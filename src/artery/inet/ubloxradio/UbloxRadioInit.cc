#include <artery/inet/ubloxradio/UbloxRadioInit.h>
#include <artery/inet/ubloxradio/ubloxradio/sim_rx_mex_initialize.h>
#include <artery/inet/ubloxradio/ubloxradio/sim_rx_mex_terminate.h>

UbloxRadioInit::UbloxRadioInit(){}

namespace {
class RInit {
public:
    RInit();
    ~RInit();
};
}

RInit::RInit() {
    sim_rx_mex_initialize();
}

RInit::~RInit() {
    sim_rx_mex_terminate();
}

RInit initer;