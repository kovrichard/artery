//
// Copyright (C) 2013 OpenSim Ltd
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

//package inet.physicallayer.ieee80211.packetlevel;
package artery.inet.ubloxradio;

import inet.physicallayer.ieee80211.packetlevel.Ieee80211TransmitterBase;

//
// This transmitter model produces IEEE 802.11 transmissions that have scalar
// transmission power in their analog representation. The bit domain, symbol
// domain, and sample domains of the transmissions are not represented.
//
// See also ~Ieee80211ScalarReceiver, ~Ieee80211ScalarRadio, and
// ~Ieee80211ScalarRadioMedium.
//
module UbloxIeee80211Transmitter extends Ieee80211TransmitterBase
{
    parameters:
        double mcs = default(2.0);
        bool window_en = default(false);
        double w_beta = default(3);
        @class(UbloxIeee80211Transmitter);
}
