#ifndef ARTERY_CPSERVICE_H_
#define ARTERY_CPSERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>

namespace artery
{

class Timer;
class VehicleDataProvider;

class CpService : public ItsG5BaseService
{
	public:
		CpService();
		void initialize() override;
		void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
		void trigger() override;

	private:
		void checkTriggeringConditions(const omnetpp::SimTime&);
		bool checkHeadingDelta() const;
		bool checkPositionDelta() const;
		bool checkSpeedDelta() const;
		void sendCam(const omnetpp::SimTime&);
		omnetpp::SimTime genCamDcc();

		const VehicleDataProvider* mVehicleDataProvider;
		const Timer* mTimer;
		artery::LocalDynamicMap* mLocalDynamicMap;
		omnetpp::SimTime mGenCamMin;
		omnetpp::SimTime mGenCamMax;
		omnetpp::SimTime mGenCam;
		unsigned mGenCamLowDynamicsCounter;
		unsigned mGenCamLowDynamicsLimit;
		Position mLastCamPosition;
		vanetza::units::Velocity mLastCamSpeed;
		vanetza::units::Angle mLastCamHeading;
		omnetpp::SimTime mLastCamTimestamp;
		omnetpp::SimTime mLastLowCamTimestamp;
		vanetza::units::Angle mHeadingDelta;
		vanetza::units::Length mPositionDelta;
		vanetza::units::Velocity mSpeedDelta;
		bool mDccRestriction;
		bool mFixedRate;
};

//vanetza::asn1::Cam createCooperativeAwarenessMessage(const VehicleDataProvider&, uint16_t genDeltaTime);
//void addLowFrequencyContainer(vanetza::asn1::Cam&);

} // namespace artery

#endif /* ARTERY_CPSERVICE_H_ */
