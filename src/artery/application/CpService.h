#ifndef ARTERY_CPSERVICE_H_
#define ARTERY_CPSERVICE_H_

#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cpm.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>
#include <omnetpp/coutvector.h>

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

		vanetza::asn1::Cpm createCollectivePerceptionMessage(const VehicleDataProvider&, const LocalEnvironmentModel&, uint16_t genDeltaTime);


		void checkTriggeringConditions(const omnetpp::SimTime&);
		bool checkHeadingDelta() const;
		bool checkPositionDelta() const;
		bool checkSpeedDelta() const;
		void sendCpm(const omnetpp::SimTime&);
		omnetpp::SimTime genCpmDcc();

		const VehicleDataProvider* mVehicleDataProvider;
		const LocalEnvironmentModel* mLocalEnvironmentModel;
		const Timer* mTimer;
		//artery::LocalDynamicMap* mLocalDynamicMap;
		omnetpp::SimTime mGenCpmMin;
		omnetpp::SimTime mGenCpmMax;
		omnetpp::SimTime mGenCpm;
		unsigned mGenCpmLowDynamicsCounter;
		unsigned mGenCpmLowDynamicsLimit;
		Position mLastCpmPosition;
		vanetza::units::Velocity mLastCpmSpeed;
		vanetza::units::Angle mLastCpmHeading;
		omnetpp::SimTime mLastCpmTimestamp;
		//omnetpp::SimTime mLastLowCpmTimestamp;
		omnetpp::SimTime mLastSensorInformationTimestamp;
		vanetza::units::Angle mHeadingDelta;
		vanetza::units::Length mPositionDelta;
		vanetza::units::Velocity mSpeedDelta;
		bool mDccRestriction;
		bool mFixedRate;
		omnetpp::cOutVector timeBetweenMessages;
		omnetpp::cOutVector sendCpmCase;
		omnetpp::cOutVector sentCpms;
		omnetpp::cOutVector receivedCpms;
		omnetpp::cOutVector radarDetectedCars;
		omnetpp::cOutVector camDetectedCars;
		omnetpp::cOutVector allDetectedCars;

		omnetpp::cOutVector CpmSize;
		
		int sentCpmCounter = 0;
		int receivedCpmCounter = 0;
};

// vanetza::asn1::Cpm createCollectivePerceptionMessage(const VehicleDataProvider&, const LocalEnvironmentModel&, uint16_t genDeltaTime);

//vanetza::asn1::Cpm createCooperativeAwarenessMessage(const VehicleDataProvider&, uint16_t genDeltaTime);
//void addLowFrequencyContainer(vanetza::asn1::Cpm&);
void addSensorInformationContainer(vanetza::asn1::Cpm&);

} // namespace artery

#endif /* ARTERY_CPSERVICE_H_ */
