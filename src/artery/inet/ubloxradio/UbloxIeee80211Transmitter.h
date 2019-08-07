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

#ifndef __INET_UBLOXIEEE80211TRANSMITTER_H
#define __INET_UBLOXIEEE80211TRANSMITTER_H

#include <inet/physicallayer/ieee80211/packetlevel/Ieee80211TransmitterBase.h>
#include <artery/inet/ubloxradio/ubloxradio/sim_rx_mex_types.h>

namespace artery {

class UbloxIeee80211Transmitter : public inet::physicallayer::Ieee80211TransmitterBase
{
  public:
    UbloxIeee80211Transmitter();

    virtual std::ostream& printToStream(std::ostream& stream, int level) const override;

    virtual const inet::physicallayer::ITransmission *createTransmission(const inet::physicallayer::IRadio *radio, const inet::Packet *packet, omnetpp::simtime_t startTime) const override;

  protected:
    virtual void initialize(int stage) override;

  private:

    std::vector<creal_T> encapsulate(const inet::Packet*) const;

    void packetToMac(const inet::Packet*, unsigned int[], int []) const;

    double mcs = 2; // Modulation and coding scheme value

    bool window_en = false; // Apply time-domain windowing

    double w_beta = 3;  // Kaiser window beta coefficient for spectral shaping (0: disabled)
};

}

#endif // ifndef __INET_UBLOXIEEE80211TRANSMITTER_H

