#include "artery/application/CpService.h"
#include "artery/application/CpObject.h"
#include "artery/application/Asn1PacketVisitor.h"
#include "artery/application/VehicleDataProvider.h"
#include "artery/utility/simtime_cast.h"
#include "veins/base/utils/Coord.h"
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/cexception.h>
#include <vanetza/btp/ports.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <chrono>


namespace artery
{

using namespace omnetpp;
/*

declared in CaService.cc*/

namespace cpservice{

auto microdegree = vanetza::units::degree * boost::units::si::micro;
auto decidegree = vanetza::units::degree * boost::units::si::deci;
auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;
}
/*
extern auto microdegree;
extern auto decidegree;
extern auto degree_per_second;
extern auto centimeter_per_second;*/

static const simsignal_t scSignalCpmReceived = cComponent::registerSignal("CpmReceived");
static const simsignal_t scSignalCpmSent = cComponent::registerSignal("CpmSent");

static const auto scSensorInformationContainerInterval = std::chrono::milliseconds(1000);


template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round(v.value());
}


Define_Module(CpService)

CpService::CpService() :
		mVehicleDataProvider(nullptr),
		mTimer(nullptr),
		mGenCpmMin { 100, SIMTIME_MS },
		mGenCpmMax { 1000, SIMTIME_MS },
		mGenCpm(mGenCpmMax),
		mGenCpmLowDynamicsCounter(0),
		mGenCpmLowDynamicsLimit(3)
{
}

void CpService::initialize()
{
	ItsG5BaseService::initialize();
	mVehicleDataProvider = &getFacilities().get_const<VehicleDataProvider>();
	mLocalEnvironmentModel = &getFacilities().get_const<LocalEnvironmentModel>();
	mTimer = &getFacilities().get_const<Timer>();
	// avoid unreasonable high elapsed time values for newly inserted vehicles
	mLastCpmTimestamp = simTime();
	mLastSensorInformationTimestamp = mLastCpmTimestamp - artery::simtime_cast(scSensorInformationContainerInterval);

	// generation rate boundaries
	mGenCpmMin = par("minInterval");
	mGenCpmMax = par("maxInterval");

	// vehicle dynamics thresholds
	mHeadingDelta = vanetza::units::Angle { par("headingDelta").doubleValue() * vanetza::units::degree };
	mPositionDelta = par("positionDelta").doubleValue() * vanetza::units::si::meter;
	mSpeedDelta = par("speedDelta").doubleValue() * vanetza::units::si::meter_per_second;

	mDccRestriction = par("withDccRestriction");
	mFixedRate = par("fixedRate");
	timeBetweenMessages.setName("timeBetweenMessages");
	sendCpmCase.setName("sendCpmCase");
	sentCpms.setName("sentCpms");
	receivedCpms.setName("receivedCpms");
	allDetectedCars.setName("allDetectedCars");
	//camDetectedCars.setName("camDetectedCars");
	radarDetectedCars.setName("radarDetectedCars");
	CpmSize.setName("CpmSize");
}

void CpService::trigger()
{
	auto allObjects = mLocalEnvironmentModel->allObjects();
	//auto camObjs = filterBySensorCategory(allObjects, "CA");
	auto radarObjs = filterBySensorCategory(allObjects, "Radar");
	
	allDetectedCars.record(allObjects.size());
	//camDetectedCars.record(boost::size(camObjs));
	radarDetectedCars.record(boost::size(radarObjs));

	//detectedCars.record(mLocalEnvironmentModel->allObjects().size());
	checkTriggeringConditions(simTime());
}

void CpService::indicate(const vanetza::btp::DataIndication& ind, std::unique_ptr<vanetza::UpPacket> packet)
{
	Asn1PacketVisitor<vanetza::asn1::Cpm> visitor;
	const vanetza::asn1::Cpm* cpm = boost::apply_visitor(visitor, *packet);
	if (cpm && cpm->validate()) {
		CpObject obj = visitor.shared_wrapper;
		receivedCpmCounter++;
		receivedCpms.record(receivedCpmCounter);
		emit(scSignalCpmReceived, &obj);
	}
}

void CpService::checkTriggeringConditions(const SimTime& T_now)
{
	// provide variables named like in EN 302 637-2 V1.3.2 (section 6.1.3)
	SimTime& T_GenCpm = mGenCpm;
	const SimTime& T_GenCpmMin = mGenCpmMin;
	const SimTime& T_GenCpmMax = mGenCpmMax;
	const SimTime T_GenCpmDcc = mDccRestriction ? genCpmDcc() : mGenCpmMin;
	const SimTime T_elapsed = T_now - mLastCpmTimestamp;

	if (T_elapsed >= T_GenCpmDcc) {
		if (mFixedRate) {
			timeBetweenMessages.record(T_elapsed);
			sendCpmCase.record(1);
			sendCpm(T_now);
		} else if (checkHeadingDelta() || checkPositionDelta() || checkSpeedDelta()) {
			timeBetweenMessages.record(T_elapsed);
			sendCpmCase.record(2);
			sendCpm(T_now);
			//T_GenCpm = std::min(T_elapsed, T_GenCpmMax); /*< if middleware update interval is too long */
			//mGenCpmLowDynamicsCounter = 0;
		} else if (T_elapsed >= T_GenCpm) {
			timeBetweenMessages.record(T_elapsed);
			sendCpmCase.record(3);
			sendCpm(T_now);
			//if (++mGenCpmLowDynamicsCounter >= mGenCpmLowDynamicsLimit) {
			//	T_GenCpm = T_GenCpmMax;
			//}
		}
	}
}

bool CpService::checkHeadingDelta() const
{
	return abs(mLastCpmHeading - mVehicleDataProvider->heading()) > mHeadingDelta;
}

bool CpService::checkPositionDelta() const
{
	return (distance(mLastCpmPosition, mVehicleDataProvider->position()) > mPositionDelta);
}

bool CpService::checkSpeedDelta() const
{
	return abs(mLastCpmSpeed - mVehicleDataProvider->speed()) > mSpeedDelta;
}

void CpService::sendCpm(const SimTime& T_now)
{
	sentCpmCounter++;
	sentCpms.record(sentCpmCounter);
	uint16_t genDeltaTimeMod = countTaiMilliseconds(mTimer->getTimeFor(mVehicleDataProvider->updated()));
	auto cpm = createCollectivePerceptionMessage(*mVehicleDataProvider, *mLocalEnvironmentModel, genDeltaTimeMod);

	mLastCpmPosition = mVehicleDataProvider->position();
	mLastCpmSpeed = mVehicleDataProvider->speed();
	mLastCpmHeading = mVehicleDataProvider->heading();
	mLastCpmTimestamp = T_now;
	/*if (T_now - mLastLowCpmTimestamp >= artery::simtime_cast(scLowFrequencyContainerInterval)) {
		addLowFrequencyContainer(cpm);
		mLastLowCpmTimestamp = T_now;
	}*/

	if (T_now - mLastSensorInformationTimestamp >= artery::simtime_cast(scSensorInformationContainerInterval)) {
		//EV_WARN << "SENSORINFORMATION " << findHost()->getIndex();
		addSensorInformationContainer(cpm);
		mLastSensorInformationTimestamp = T_now;
	}
	using namespace vanetza;
	btp::DataRequestB request;
	request.destination_port = btp::ports::CPM;
	request.gn.its_aid = aid::CA;
	request.gn.transport_type = geonet::TransportType::SHB;
	request.gn.maximum_lifetime = geonet::Lifetime { geonet::Lifetime::Base::One_Second, 1 };
	request.gn.traffic_class.tc_id(static_cast<unsigned>(dcc::Profile::DP2));
	request.gn.communication_profile = geonet::CommunicationProfile::ITS_G5;

	CpObject obj(std::move(cpm));
	emit(scSignalCpmSent, &obj);

	using CpmByteBuffer = convertible::byte_buffer_impl<asn1::Cpm>;
	std::unique_ptr<geonet::DownPacket> payload { new geonet::DownPacket() };
	std::unique_ptr<convertible::byte_buffer> buffer { new CpmByteBuffer(obj.shared_ptr()) };
	payload->layer(OsiLayer::Application) = std::move(buffer);
	EV_WARN << payload->size() << "\n";
	CpmSize.record(payload->size());

	this->request(request, std::move(payload));
}

SimTime CpService::genCpmDcc()
{
	static const vanetza::dcc::TransmissionLite cp_tx(vanetza::dcc::Profile::DP2, 0);
	auto& trc = getFacilities().get_mutable<vanetza::dcc::TransmitRateThrottle>();
	vanetza::Clock::duration delay = trc.delay(cp_tx);
	SimTime dcc { std::chrono::duration_cast<std::chrono::milliseconds>(delay).count(), SIMTIME_MS };
	return std::min(mGenCpmMax, std::max(mGenCpmMin, dcc));
}

vanetza::asn1::Cpm CpService::createCollectivePerceptionMessage(const VehicleDataProvider& vdp, const LocalEnvironmentModel& lem, uint16_t genDeltaTime)
{
	const auto& objects = lem.allObjects();
	unsigned objectsCount = objects.size();

	const auto* vehicleController = getFacilities().get_const_ptr<traci::VehicleController>();
	//EV_INFO << "VEHICLE LENGTH: " << vehicleController->getLength().value();

	//EV_INFO << "Objects count (node " << findHost()->getIndex() << "): " << objectsCount << "\n";

	vanetza::asn1::Cpm message;

	ItsPduHeader_t& header = (*message).header;
	header.protocolVersion = 1;
	header.messageID = ItsPduHeader__messageID_tistpgtransaction;
	header.stationID = vdp.station_id();

	CollectivePerceptionMessage_t& cpm = (*message).cpm;
	cpm.generationDeltaTime = genDeltaTime * GenerationDeltaTime_oneMilliSec;
	/*BasicContainer_t& basic = cpm.cpmParameters.basicContainer;
	HighFrequencyContainer_t& hfc = cpm.cpmParameters.highFrequencyContainer;

	basic.stationType = StationType_passengerCar;
	basic.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
	basic.referencePosition.longitude = round(vdp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;
	basic.referencePosition.latitude = round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
	basic.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence =
			SemiAxisLength_unavailable;
	basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence =
			SemiAxisLength_unavailable;

	hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
	BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;
	bvc.heading.headingValue = round(vdp.heading(), decidegree);
	bvc.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
	bvc.speed.speedValue = round(vdp.speed(), centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
	bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
	bvc.driveDirection = vdp.speed().value() >= 0.0 ?
			DriveDirection_forward : DriveDirection_backward;
	const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
	// extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
	if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
	} else {
		bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
	}
	bvc.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
	bvc.curvature.curvatureValue = abs(vdp.curvature() / vanetza::units::reciprocal_metre) * 10000.0;
	if (bvc.curvature.curvatureValue >= 1023) {
		bvc.curvature.curvatureValue = 1023;
	}
	bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
	bvc.curvatureCalculationMode = CurvatureCalculationMode_yawRateUsed;
	bvc.yawRate.yawRateValue = round(vdp.yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0;
	if (abs(bvc.yawRate.yawRateValue) >= YawRateValue_unavailable) {
		bvc.yawRate.yawRateValue = YawRateValue_unavailable;
	}
	bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
	bvc.vehicleLength.vehicleLengthConfidenceIndication =
			VehicleLengthConfidenceIndication_noTrailerPresent;
	bvc.vehicleWidth = VehicleWidth_unavailable;*/
	using namespace cpservice;

	CpmManagementContainer_t& managementContainer = cpm.cpmParameters.managementContainer;

	managementContainer.stationType = StationType_passengerCar;
	managementContainer.referencePosition.latitude = round(vdp.latitude(), microdegree) * Latitude_oneMicrodegreeNorth;
	managementContainer.referencePosition.longitude = round(vdp.longitude(), microdegree) * Longitude_oneMicrodegreeEast;
	managementContainer.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
	managementContainer.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
	managementContainer.referencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
	managementContainer.referencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
	managementContainer.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;

	cpm.cpmParameters.stationDataContainer = vanetza::asn1::allocate<StationDataContainer_t>();
	StationDataContainer_t*& stationDataContainer = cpm.cpmParameters.stationDataContainer;
	stationDataContainer->present = StationDataContainer_PR_originatingVehicleContainer;
	OriginatingVehicleContainer_t& originatingVehicleContainer = stationDataContainer->choice.originatingVehicleContainer;
	originatingVehicleContainer.heading.headingValue = round(vdp.heading(), decidegree);
	originatingVehicleContainer.heading.headingConfidence = HeadingConfidence_equalOrWithinOneDegree;
	originatingVehicleContainer.speed.speedValue = round(vdp.speed(), centimeter_per_second) * SpeedValue_oneCentimeterPerSec;
	originatingVehicleContainer.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;

	originatingVehicleContainer.vehicleOrientationAngle = vanetza::asn1::allocate<WGS84Angle_t>();
	originatingVehicleContainer.vehicleOrientationAngle->value = round(vdp.heading(), decidegree);
	originatingVehicleContainer.vehicleOrientationAngle->confidence = AngleConfidence_zeroPointOneDegree;
	originatingVehicleContainer.driveDirection = vdp.speed().value() >= 0.0 ?
			DriveDirection_forward : DriveDirection_backward;

	originatingVehicleContainer.longitudinalAcceleration = vanetza::asn1::allocate<LongitudinalAcceleration_t>();

	const double lonAccelValue = vdp.acceleration() / vanetza::units::si::meter_per_second_squared;
	// extreme speed changes can occur when SUMO swaps vehicles between lanes (speed is swapped as well)
	if (lonAccelValue >= -160.0 && lonAccelValue <= 161.0) {
		originatingVehicleContainer.longitudinalAcceleration->longitudinalAccelerationValue = lonAccelValue * LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
	} else {
		originatingVehicleContainer.longitudinalAcceleration->longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;
	}

	originatingVehicleContainer.longitudinalAcceleration->longitudinalAccelerationConfidence = AccelerationConfidence_pointOneMeterPerSecSquared;
	
	originatingVehicleContainer.lateralAcceleration = vanetza::asn1::allocate<LateralAcceleration_t>();
	originatingVehicleContainer.lateralAcceleration->lateralAccelerationValue = LateralAccelerationValue_unavailable;
	originatingVehicleContainer.lateralAcceleration->lateralAccelerationConfidence = AccelerationConfidence_unavailable;
	originatingVehicleContainer.yawRate = vanetza::asn1::allocate<YawRate_t>();
	originatingVehicleContainer.yawRate->yawRateValue = round(vdp.yaw_rate(), degree_per_second) * YawRateValue_degSec_000_01ToLeft * 100.0;
	if (abs(originatingVehicleContainer.yawRate->yawRateValue) >= YawRateValue_unavailable) {
		originatingVehicleContainer.yawRate->yawRateValue = YawRateValue_unavailable;
	}
	originatingVehicleContainer.yawRate->yawRateConfidence = YawRateConfidence_degSec_000_01;

	//cpm.cpmParameters.sensorInformationContainer = vanetza::asn1::allocate<CpmParameters::CpmParameters__sensorInformationContainer>();
	//cpm.cpmParameters.sensorInformationContainer->list

	/*cpm.cpmParameters.sensorInformationContainer = vanetza::asn1::allocate<CpmParameters::CpmParameters__sensorInformationContainer>();

	SensorInformation* sensorInformation = vanetza::asn1::allocate<SensorInformation>();
	sensorInformation->id = 0;
	sensorInformation->type = SensorType_radar;
	sensorInformation->detectionArea.present = DetectionArea_PR_stationarySensorCircular;
	AreaCircular_t& stationarySensorCircular = sensorInformation->detectionArea.choice.stationarySensorCircular;
	stationarySensorCircular.nodeCenterPoint = vanetza::asn1::allocate<OffsetPoint>();
	stationarySensorCircular.nodeCenterPoint->nodeOffsetPointxy.present = NodeOffsetPointXYEU_PR_node_XY1;
	stationarySensorCircular.nodeCenterPoint->nodeOffsetPointxy.choice.node_XY1.x = 1;
	stationarySensorCircular.nodeCenterPoint->nodeOffsetPointxy.choice.node_XY1.y = 1;
	stationarySensorCircular.radius = Radius_oneMeter;
	sensorInformation->freeSpaceConfidence = vanetza::asn1::allocate<FreeSpaceConfidence_t>();
	*(sensorInformation->freeSpaceConfidence) = FreeSpaceConfidence_unitConfidenceLevel;

	ASN_SEQUENCE_ADD(cpm.cpmParameters.sensorInformationContainer, sensorInformation);*/


	cpm.cpmParameters.numberOfPerceivedObjects = objectsCount;

	if (objectsCount > 33)
		objectsCount = 33;


	if (objectsCount > 0){

		cpm.cpmParameters.perceivedObjectContainer = vanetza::asn1::allocate<CpmParameters::CpmParameters__perceivedObjectContainer>();

		for (int i = 0; i < objectsCount; ++i) {
			//EV_WARN << i << "\n";
		
			PerceivedObject* perceivedObject = vanetza::asn1::allocate<PerceivedObject>();

			perceivedObject->objectID = i;

			Identifier_t* identifier = vanetza::asn1::allocate<Identifier_t>();
			*identifier = 1;
			ASN_SEQUENCE_ADD(perceivedObject->sensorID, identifier);

			perceivedObject->timeOfMeasurement = TimeOfMeasurement_oneMilliSecond;
			perceivedObject->objectAge = vanetza::asn1::allocate<ObjectAge_t>();
			*(perceivedObject->objectAge) = ObjectAge_oneMiliSec;

			perceivedObject->objectConfidence = vanetza::asn1::allocate<ObjectConfidence_t>();
			*(perceivedObject->objectConfidence) = 100;

			perceivedObject->xDistance.value = DistanceValue_zeroPointZeroOneMeter;
			perceivedObject->xDistance.confidence = DistanceConfidence_zeroPointZeroOneMeter;
			perceivedObject->yDistance.value = DistanceValue_zeroPointZeroOneMeter;
			perceivedObject->yDistance.confidence = DistanceConfidence_zeroPointZeroOneMeter;
			perceivedObject->xSpeed.value = SpeedValue_oneCentimeterPerSec;
			perceivedObject->xSpeed.confidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec;
			perceivedObject->ySpeed.value = SpeedValue_oneCentimeterPerSec;
			perceivedObject->ySpeed.confidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec;
			perceivedObject->xAcceleration = vanetza::asn1::allocate<LongitudinalAcceleration_t>();
			perceivedObject->xAcceleration->longitudinalAccelerationValue = LongitudinalAccelerationValue_pointOneMeterPerSecSquaredForward;
			perceivedObject->xAcceleration->longitudinalAccelerationConfidence = LongitudinalLanePositionConfidence_zeroPointZeroOneMeter;
			perceivedObject->objectRefPoint = ObjectRefPoint_mid;
			perceivedObject->dynamicStatus = vanetza::asn1::allocate<DynamicStatus_t>();
			*(perceivedObject->dynamicStatus) = DynamicStatus_dynamic;
			
			perceivedObject->classification = vanetza::asn1::allocate<ObjectClassDescription_t>();

			for (int j = 0; j < 2; ++j) {

				ObjectClass* objectClass = vanetza::asn1::allocate<ObjectClass>();
				objectClass->confidence = 50;
				objectClass->Class.present = ObjectClass__class_PR_vehicle;
				objectClass->Class.choice.vehicle.confidence = vanetza::asn1::allocate<ClassConfidence_t>();
				*(objectClass->Class.choice.vehicle.confidence) = 100;
				objectClass->Class.choice.vehicle.type = vanetza::asn1::allocate<long>();
				*(objectClass->Class.choice.vehicle.type);
				ASN_SEQUENCE_ADD(perceivedObject->classification, objectClass);
			}

			ASN_SEQUENCE_ADD(cpm.cpmParameters.perceivedObjectContainer, perceivedObject);
		}
/*
		cpm.cpmParameters.freeSpaceAddendum = vanetza::asn1::allocate<CpmParameters::CpmParameters__freeSpaceAddendum>();

		if(objectsCount > 29)
			objectsCount = 29;

		for (int i = 0; i < objectsCount; ++i){
			
			FreeSpaceAddendum* freeSpaceAddendum = vanetza::asn1::allocate<FreeSpaceAddendum>();

			freeSpaceAddendum->freeSpaceConfidence = 100;
			freeSpaceAddendum->freeSpaceArea.present = FreeSpaceArea_PR_freeSpaceRadial;
			AreaRadial_t& freeSpaceRadial = freeSpaceAddendum->freeSpaceArea.choice.freeSpaceRadial;
			freeSpaceRadial.range = 75 * Range_oneMeter;
			freeSpaceRadial.stationaryHorizontalOpeningAngleStart = WGS84AngleValue_wgs84North;
			freeSpaceRadial.stationaryHorizontalOpeningAngleEnd = WGS84AngleValue_wgs84North;
			freeSpaceRadial.sensorPositionOffset = vanetza::asn1::allocate<OffsetPoint>();
			freeSpaceRadial.sensorPositionOffset->nodeOffsetPointxy.present = NodeOffsetPointXYEU_PR_node_XY1;
			freeSpaceRadial.sensorPositionOffset->nodeOffsetPointxy.choice.node_XY1.x = -1 * (vehicleController->getLength().value() * 100) / 2;
			freeSpaceRadial.sensorPositionOffset->nodeOffsetPointxy.choice.node_XY1.y = 0;
			
			freeSpaceAddendum->sensorID = vanetza::asn1::allocate<FreeSpaceAddendum::FreeSpaceAddendum__sensorID>();

			Identifier_t* identifier = vanetza::asn1::allocate<Identifier_t>();
			*identifier = 0;
			ASN_SEQUENCE_ADD(freeSpaceAddendum->sensorID, identifier);

			ASN_SEQUENCE_ADD(cpm.cpmParameters.freeSpaceAddendum, freeSpaceAddendum);
		}*/

	}
	cpm.cpmParameters.freeSpaceAddendum = vanetza::asn1::allocate<CpmParameters::CpmParameters__freeSpaceAddendum>();

		FreeSpaceAddendum* freeSpaceAddendum = vanetza::asn1::allocate<FreeSpaceAddendum>();
		freeSpaceAddendum->freeSpaceConfidence = 100;
		freeSpaceAddendum->freeSpaceArea.present = FreeSpaceArea_PR_freeSpacePolygon;
		AreaPolygon_t& freeSpacePoligon = freeSpaceAddendum->freeSpaceArea.choice.freeSpacePolygon;
		
		for (int i = 0; i < 16; ++i){
			OffsetPoint* offsetPoint = vanetza::asn1::allocate<OffsetPoint_t>();
			offsetPoint->nodeOffsetPointxy.present = NodeOffsetPointXYEU_PR_node_XY6;
			offsetPoint->nodeOffsetPointxy.choice.node_XY6.x = 1;
			offsetPoint->nodeOffsetPointxy.choice.node_XY6.y = 1;
			ASN_SEQUENCE_ADD(&freeSpacePoligon.polyPoint, offsetPoint);
		}

		freeSpaceAddendum->sensorID = vanetza::asn1::allocate<FreeSpaceAddendum::FreeSpaceAddendum__sensorID>();

		Identifier_t* identifier = vanetza::asn1::allocate<Identifier_t>();
		*identifier = 0;
		ASN_SEQUENCE_ADD(freeSpaceAddendum->sensorID, identifier);

		ASN_SEQUENCE_ADD(cpm.cpmParameters.freeSpaceAddendum, freeSpaceAddendum);

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid CPM: %s", error.c_str());
	}

	return message;
}
void addSensorInformationContainer(vanetza::asn1::Cpm& message)
{
	message->cpm.cpmParameters.sensorInformationContainer = vanetza::asn1::allocate<CpmParameters::CpmParameters__sensorInformationContainer>();

	for(int i = 0; i < 2; i++){

	SensorInformation* sensorInformation = vanetza::asn1::allocate<SensorInformation>();
	sensorInformation->id = 0;
	sensorInformation->type = SensorType_radar;
	/*sensorInformation->detectionArea.present = DetectionArea_PR_stationarySensorCircular;
	AreaCircular_t& stationarySensorCircular = sensorInformation->detectionArea.choice.stationarySensorCircular;
	stationarySensorCircular.nodeCenterPoint = vanetza::asn1::allocate<OffsetPoint>();
	stationarySensorCircular.nodeCenterPoint->nodeOffsetPointxy.present = NodeOffsetPointXYEU_PR_node_XY1;
	stationarySensorCircular.nodeCenterPoint->nodeOffsetPointxy.choice.node_XY1.x = 1;
	stationarySensorCircular.nodeCenterPoint->nodeOffsetPointxy.choice.node_XY1.y = 1;
	stationarySensorCircular.radius = Radius_oneMeter;*/
	sensorInformation->detectionArea.present = DetectionArea_PR_stationarySensorRadial;
	AreaRadial_t& stationarySensorRadial = sensorInformation->detectionArea.choice.stationarySensorRadial;
	stationarySensorRadial.range = Range_zeroPointOneMeter;
	stationarySensorRadial.sensorPositionOffset = vanetza::asn1::allocate<OffsetPoint_t>();
	stationarySensorRadial.sensorPositionOffset->nodeOffsetPointxy.present = NodeOffsetPointXYEU_PR_node_XY1;
	stationarySensorRadial.sensorPositionOffset->nodeOffsetPointxy.choice.node_XY1.x = 1;
	stationarySensorRadial.sensorPositionOffset->nodeOffsetPointxy.choice.node_XY1.y = 1;
	stationarySensorRadial.stationaryHorizontalOpeningAngleEnd = WGS84AngleValue_unavailable;
	stationarySensorRadial.stationaryHorizontalOpeningAngleStart = WGS84AngleValue_unavailable;
	
 	sensorInformation->freeSpaceConfidence = vanetza::asn1::allocate<FreeSpaceConfidence_t>();
	*(sensorInformation->freeSpaceConfidence) = FreeSpaceConfidence_unitConfidenceLevel;

	ASN_SEQUENCE_ADD(message->cpm.cpmParameters.sensorInformationContainer, sensorInformation);
	}

}
/*void addLowFrequencyContainer(vanetza::asn1::Cpm& message)
{
	LowFrequencyContainer_t*& lfc = message->cpm.cpmParameters.lowFrequencyContainer;
	lfc = vanetza::asn1::allocate<LowFrequencyContainer_t>();
	lfc->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
	BasicVehicleContainerLowFrequency& bvc = lfc->choice.basicVehicleContainerLowFrequency;
	bvc.vehicleRole = VehicleRole_default;
	bvc.exteriorLights.buf = static_cast<uint8_t*>(vanetza::asn1::allocate(1));
	assert(nullptr != bvc.exteriorLights.buf);
	bvc.exteriorLights.size = 1;
	bvc.exteriorLights.buf[0] |= 1 << (7 - ExteriorLights_daytimeRunningLightsOn);
	// TODO: add pathHistory

	std::string error;
	if (!message.validate(error)) {
		throw cRuntimeError("Invalid Low Frequency CAM: %s", error.c_str());
	}
}*/

} // namespace artery
