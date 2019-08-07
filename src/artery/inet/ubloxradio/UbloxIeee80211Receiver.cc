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

#include <inet/physicallayer/base/packetlevel/FlatTransmissionBase.h>
#include <artery/inet/ubloxradio/UbloxIeee80211Receiver.h>
#include <artery/inet/ubloxradio/ubloxradio/sim_rx.h>
#include <artery/inet/ubloxradio/UbloxRadioInit.h>
#include <artery/inet/ubloxradio/ubloxradio/sim_rx_mex_emxAPI.h>
#include <artery/inet/ubloxradio/ubloxradio/sim_rx_mex_emxutil.h>
#include <artery/inet/ubloxradio/ubloxradio/add_impairments.h>
#include <artery/inet/ubloxradio/ubloxradio/sim_rx.h>
#include <inet/physicallayer/ieee80211/packetlevel/Ieee80211ScalarTransmission.h>
#include <cmath>

#include <artery/inet/ubloxradio/UbloxTransmission.h>

namespace artery
{

Define_Module(UbloxIeee80211Receiver);

UbloxIeee80211Receiver::UbloxIeee80211Receiver() :
    Ieee80211ReceiverBase()
{
    UbloxRadioInit();
}

void UbloxIeee80211Receiver::initialize(int stage)
{
    Ieee80211ReceiverBase::initialize(stage);
    if (stage == inet::INITSTAGE_LOCAL) {
        snr = par("snr").doubleValue();
        apply_cfo = par("apply_cfo").boolValue();
        pdet_thold = par("pdet_thold").doubleValue();
    }
}

std::ostream& UbloxIeee80211Receiver::printToStream(std::ostream& stream, int level) const
{
    stream << "UbloxIeee80211Receiver";
    return inet::physicallayer::Ieee80211ReceiverBase::printToStream(stream, level);
}

bool UbloxIeee80211Receiver::computeIsReceptionPossible(const inet::physicallayer::IListening *listening, const inet::physicallayer::ITransmission *transmission) const
{
    bool cond = true;
    if(!decapsulate(transmission)) { // TODO somehow else
        cond = false;
    }
    auto ieee80211Transmission = dynamic_cast<const inet::physicallayer::Ieee80211ScalarTransmission *>(transmission);
    return ieee80211Transmission && modeSet->containsMode(ieee80211Transmission->getMode()) && inet::physicallayer::NarrowbandReceiverBase::computeIsReceptionPossible(listening, transmission) && cond;
}

bool UbloxIeee80211Receiver::computeIsReceptionPossible(const inet::physicallayer::IListening *listening, const inet::physicallayer::IReception *reception, inet::physicallayer::IRadioSignal::SignalPart part) const
{
    bool cond = true;
    if(!decapsulate(reception->getTransmission())) {
        cond = false;
    }
    auto ieee80211Transmission = dynamic_cast<const inet::physicallayer::Ieee80211ScalarTransmission *>(reception->getTransmission());
    return ieee80211Transmission && modeSet->containsMode(ieee80211Transmission->getMode()) && inet::physicallayer::FlatReceiverBase::computeIsReceptionPossible(listening, reception, part) && cond;
}

bool UbloxIeee80211Receiver::decapsulate(const inet::physicallayer::ITransmission * transmission/* const inet::Packet* packet*/) const 
{
    emxArray_creal_T *rx_wf;
    struct1_T SIM;
    emxArray_creal_T *tx_wf;
    double s0_len;
    emxInitArray_creal_T(&rx_wf, 1);
    emxInitArray_creal_T(&tx_wf, 1);
       /* Initialize function 'add_impairments' input arguments. */
    /* Initialize function input argument 'SIM'. */
    SIM.snr = snr;
    SIM.use_mex = false;
    SIM.apply_cfo = apply_cfo;
       /* Initialize function input argument 'tx_wf'. */

    auto ubloxTransmission = dynamic_cast<const artery::UbloxTransmission *>(transmission);

    if(ubloxTransmission == nullptr) {
        emxDestroyArray_creal_T(tx_wf);
        emxDestroyArray_creal_T(rx_wf);
        return false;
    }

    std::vector<creal_T> wf_data = ubloxTransmission->getWf();

    emxEnsureCapacity_creal_T(tx_wf, 1);
    tx_wf->size[0] = wf_data.size();
    tx_wf->size[1] = 1;
    emxEnsureCapacity_creal_T(tx_wf, wf_data.size());

    for(int i = 0; i < wf_data.size(); ++i) {
        tx_wf->data[i] = wf_data[i];
    }

       /* Call the entry-point 'add_impairments'. */
    add_impairments(&SIM, tx_wf, rx_wf, &s0_len);

    emxArray_real_T *pld_bytes;
    double err;
    emxInitArray_real_T(&pld_bytes, 2);

    /* Call the entry-point 'sim_rx'. */
    sim_rx(rx_wf, s0_len, pdet_thold, pld_bytes, &err);
    emxDestroyArray_creal_T(tx_wf);
    // TODO convert to normal packet (pld_bytes)

    emxDestroyArray_real_T(pld_bytes);
    emxDestroyArray_creal_T(rx_wf);

    return err <= 0.1 && err >= -0.1;
}

} // namespace artery