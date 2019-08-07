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

#ifndef __INET_UBLOXIEEE80211RECEIVER_H
#define __INET_UBLOXIEEE80211RECEIVER_H

#include <inet/physicallayer/ieee80211/mode/Ieee80211ModeSet.h>
#include <inet/physicallayer/ieee80211/packetlevel/Ieee80211ReceiverBase.h>

namespace artery {

class UbloxIeee80211Receiver : public inet::physicallayer::Ieee80211ReceiverBase
{
  private:
    bool decapsulate(const inet::physicallayer::ITransmission */*const inet::Packet* */) const;

    double snr = 20;  //SNR value (dB)
    bool apply_cfo = false; // Apply CFO impairment on Tx and Rx
    double pdet_thold = 20; // Packet detection threshold
    //double t_depth = 2; // Channel tracking time depth averaging (OFDM symbols)
  protected:
    virtual bool computeIsReceptionPossible(const inet::physicallayer::IListening *listening, const inet::physicallayer::ITransmission *transmission) const override;
    virtual bool computeIsReceptionPossible(const inet::physicallayer::IListening *listening, const inet::physicallayer::IReception *reception, inet::physicallayer::IRadioSignal::SignalPart part) const override;
    virtual void initialize(int stage) override;
  public:
    UbloxIeee80211Receiver();

    virtual std::ostream& printToStream(std::ostream& stream, int level) const override;
};

} // namespace artery

#endif // ifndef __INET_UBLOXIEEE80211RECEIVER_H

