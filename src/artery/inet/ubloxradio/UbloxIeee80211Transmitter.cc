//
// Copyright (C) 2013 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include <inet/mobility/contract/IMobility.h>
#include <inet/physicallayer/contract/packetlevel/IRadio.h>
#include <inet/physicallayer/contract/packetlevel/RadioControlInfo_m.h>
#include <inet/physicallayer/ieee80211/packetlevel/Ieee80211PhyHeader_m.h>
#include <inet/physicallayer/ieee80211/packetlevel/Ieee80211ScalarTransmission.h>
#include <artery/inet/ubloxradio/UbloxIeee80211Transmitter.h>
#include <artery/inet/ubloxradio/ubloxradio/sim_tx.h>
#include <artery/inet/ubloxradio/ubloxradio/sim_rx_mex_emxAPI.h>
#include <artery/inet/ubloxradio/ubloxradio/sim_rx_mex_emxutil.h>
#include <artery/inet/ubloxradio/UbloxRadioInit.h>
#include <artery/inet/ubloxradio/UbloxTransmission.h>

#include <iomanip>
#include <inet/common/Ptr.h>
#include <inet/common/MemoryOutputStream.h>
#include <random>

namespace artery {

Define_Module(UbloxIeee80211Transmitter);

using namespace inet;

UbloxIeee80211Transmitter::UbloxIeee80211Transmitter() :
    Ieee80211TransmitterBase()
{
    UbloxRadioInit();
}

void UbloxIeee80211Transmitter::initialize(int stage)
{
    Ieee80211TransmitterBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        mcs = par("mcs").doubleValue();
        window_en = par("window_en").boolValue();
        w_beta = par("w_beta").doubleValue();
    }
}

std::ostream& UbloxIeee80211Transmitter::printToStream(std::ostream& stream, int level) const
{
    stream << "UbloxIeee80211Transmitter";
    return Ieee80211TransmitterBase::printToStream(stream, level);
}

const inet::physicallayer::ITransmission *UbloxIeee80211Transmitter::createTransmission(const inet::physicallayer::IRadio *transmitter, const Packet *packet, simtime_t startTime) const
{
    auto phyHeader = packet->peekAtFront<inet::physicallayer::Ieee80211PhyHeader>();
    const inet::physicallayer::IIeee80211Mode *transmissionMode = computeTransmissionMode(packet);
    const inet::physicallayer::Ieee80211Channel *transmissionChannel = computeTransmissionChannel(packet);
    W transmissionPower = computeTransmissionPower(packet);
    auto wfVector = encapsulate(packet);
    bps transmissionBitrate = transmissionMode->getDataMode()->getNetBitrate();
    if (transmissionMode->getDataMode()->getNumberOfSpatialStreams() > transmitter->getAntenna()->getNumAntennas())
        throw cRuntimeError("Number of spatial streams is higher than the number of antennas");
    const simtime_t duration = transmissionMode->getDuration(B(phyHeader->getLengthField()));
    const simtime_t endTime = startTime + duration;
    IMobility *mobility = transmitter->getAntenna()->getMobility();
    const Coord startPosition = mobility->getCurrentPosition();
    const Coord endPosition = mobility->getCurrentPosition();
    const EulerAngles startOrientation = mobility->getCurrentAngularPosition();
    const EulerAngles endOrientation = mobility->getCurrentAngularPosition();
    auto headerLength = b(transmissionMode->getHeaderMode()->getLength());
    auto dataLength = b(transmissionMode->getDataMode()->getCompleteLength(B(phyHeader->getLengthField())));
    const simtime_t preambleDuration = transmissionMode->getPreambleMode()->getDuration();
    const simtime_t headerDuration = transmissionMode->getHeaderMode()->getDuration();
    const simtime_t dataDuration = duration - headerDuration - preambleDuration;

    return new UbloxTransmission(transmitter, packet, startTime, endTime, preambleDuration, headerDuration, dataDuration, startPosition, endPosition, startOrientation, endOrientation, modulation, headerLength, dataLength, carrierFrequency, bandwidth, transmissionBitrate, transmissionPower, transmissionMode, transmissionChannel, wfVector);
}

void UbloxIeee80211Transmitter::packetToMac(const Packet* packet, unsigned int* mac_payload_data, int* mac_payload_size) const {
    auto length = packet->getTotalLength().get() / 8; //bytelength
    
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); 
    std::uniform_int_distribution<> dis(0, 255);
    for(int i = 0; i < length; i++) {
        mac_payload_data[i] = dis(gen);
    }

    mac_payload_size[0] = 1;
    mac_payload_size[1] = length;
}

std::vector<creal_T> UbloxIeee80211Transmitter::encapsulate(const Packet* packet) const {
    emxArray_creal_T *tx_wf;
    emxArray_creal_T *data_f_mtx;
    unsigned int mac_payload_data[4096];
    int mac_payload_size[2];
    boolean_T bv1[7];
    boolean_T data_msg_data[32768];
    int data_msg_size[1];
    struct0_T PHY;
    emxInitArray_creal_T(&tx_wf, 1);
    emxInitArray_creal_T(&data_f_mtx, 2);

    /* Initialize function input argument 'mac_payload'. */
    packetToMac(packet, mac_payload_data, mac_payload_size);
    /* Initialize function input argument 'pn_seq'. */
    /* Call the entry-point 'sim_tx'. */
    for (int idx0 = 0; idx0 < 7; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
        bv1[idx0] = idx0 % 2 == 1;
    }
    sim_tx(mcs, mac_payload_data, mac_payload_size, bv1, window_en,
           w_beta, tx_wf, data_f_mtx, data_msg_data, data_msg_size, &PHY);

    std::vector<creal_T> wfVector;

    for(int i = 0; i < tx_wf->size[0]; i++) {
        wfVector.push_back(tx_wf->data[i]);
    }

    emxDestroyArray_creal_T(data_f_mtx);
    emxDestroyArray_creal_T(tx_wf);

    return wfVector;
}

}