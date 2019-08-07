#include <artery/inet/ubloxradio/UbloxTransmission.h>

namespace artery {

using namespace inet::units::values;

UbloxTransmission::UbloxTransmission(const inet::physicallayer::IRadio *transmitter, const inet::Packet *packet, const omnetpp::simtime_t startTime, const omnetpp::simtime_t endTime, const omnetpp::simtime_t preambleDuration, const omnetpp::simtime_t headerDuration, const omnetpp::simtime_t dataDuration, const inet::Coord startPosition, const inet::Coord endPosition, const inet::EulerAngles startOrientation, const inet::EulerAngles endOrientation, const inet::physicallayer::IModulation *modulation, inet::b headerLength, inet::b dataLength, Hz carrierFrequency, Hz bandwidth, bps bitrate, W power, const inet::physicallayer::IIeee80211Mode *mode, const inet::physicallayer::Ieee80211Channel *channel, std::vector<creal_T> wfs) :
    Ieee80211ScalarTransmission(transmitter, packet, startTime, endTime, preambleDuration, headerDuration, dataDuration, startPosition, endPosition, startOrientation, endOrientation, modulation, headerLength, dataLength, carrierFrequency, bandwidth, bitrate, power, mode, channel),
    wfs(wfs)
{
}

std::ostream& UbloxTransmission::printToStream(std::ostream& stream, int level) const
{
    stream << "Ieee80211ScalarTransmission";
    Ieee80211TransmissionBase::printToStream(stream, level);
    if (level <= PRINT_LEVEL_DETAIL)
       stream << ", power = " << power;
    return FlatTransmissionBase::printToStream(stream, level);
}

} // namespace physicallayer
