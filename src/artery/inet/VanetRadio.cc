/*
* Artery V2X Simulation Framework
* Copyright 2019 Raphael Riebl
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#include "artery/inet/VanetRadio.h"

namespace phy = inet::physicallayer;

namespace artery
{

Define_Module(VanetRadio)

const omnetpp::simsignal_t VanetRadio::RadioFrameSignal = omnetpp::cComponent::registerSignal("RadioFrame");

void VanetRadio::handleSignal(inet::physicallayer::Signal* frame)
{
    phy::Ieee80211Radio::handleSignal(frame);
    emit(RadioFrameSignal, frame);
}

} // namespace artery
