#ifndef __ARTERY_UBLOXTRANSMISSION_H
#define __ARTERY_UBLOXTRANSMISSION_H

#include "inet/physicallayer/analogmodel/packetlevel/ScalarTransmission.h"
#include "inet/physicallayer/ieee80211/packetlevel/Ieee80211TransmissionBase.h"
#include "inet/physicallayer/ieee80211/packetlevel/Ieee80211ScalarTransmission.h"
#include <artery/inet/ubloxradio/ubloxradio/tmwtypes.h>
#include <vector>

namespace artery {

using namespace inet::units::values;

class UbloxTransmission : public inet::physicallayer::Ieee80211ScalarTransmission
{
  private:
    std::vector<creal_T> wfs;

  public:
    UbloxTransmission(const inet::physicallayer::IRadio *transmitter, const inet::Packet *packet, const omnetpp::simtime_t startTime, const omnetpp::simtime_t endTime, const omnetpp::simtime_t preambleDuration, const omnetpp::simtime_t headerDuration, const omnetpp::simtime_t dataDuration, const inet::Coord startPosition, const inet::Coord endPosition, const inet::EulerAngles startOrientation, const inet::EulerAngles endOrientation, const inet::physicallayer::IModulation *modulation, inet::b headerLength, inet::b dataLength, Hz carrierFrequency, Hz bandwidth, bps bitrate, W power, const inet::physicallayer::IIeee80211Mode *mode, const inet::physicallayer::Ieee80211Channel *channel, std::vector<creal_T> wfs);

    std::vector<creal_T> getWf() const {return wfs;}

    virtual std::ostream& printToStream(std::ostream& stream, int level) const override;
};

} // namespace physicallayer

#endif
