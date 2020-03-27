#include <artery/inet/AntennaMobility.h>
#include <omnetpp/cexception.h>

namespace artery
{

Define_Module(AntennaMobility)

void AntennaMobility::initialize(int stage)
{
    omnetpp::cModule* module = getModuleByPath(par("mobilityModule"));
    mParentMobility = omnetpp::check_and_cast<inet::IMobility*>(module);

    mOffsetCoord.x = par("offsetX");
    mOffsetCoord.y = par("offsetY");
    mOffsetCoord.z = par("offsetZ");
    auto alpha = inet::rad(par("offsetAlpha"));
    auto beta = inet::rad(par("offsetBeta"));
    auto gamma = inet::rad(par("offsetGamma"));
    mOffsetAngles = inet::Quaternion(inet::EulerAngles(alpha, beta, gamma));
    mOffsetRotation = inet::RotationMatrix(mOffsetAngles.toEulerAngles());
}

int AntennaMobility::numInitStages() const
{
    return 1;
}

double AntennaMobility::getMaxSpeed() const
{
    return mParentMobility->getMaxSpeed();
}

inet::Coord AntennaMobility::getCurrentPosition()
{
    inet::EulerAngles angular_pos = mParentMobility->getCurrentAngularPosition().toEulerAngles();
    std::swap(angular_pos.alpha, angular_pos.gamma);
    inet::RotationMatrix rot(angular_pos);
    inet::Coord rotated_offset = rot.rotateVector(mOffsetCoord);
    return mParentMobility->getCurrentPosition() + rotated_offset;
}

inet::Coord AntennaMobility::getCurrentVelocity()
{
    return mOffsetRotation.rotateVector(mParentMobility->getCurrentVelocity());
}

inet::Coord AntennaMobility::getCurrentAcceleration()
{
    return mOffsetRotation.rotateVector(mParentMobility->getCurrentAcceleration());
}

inet::Quaternion AntennaMobility::getCurrentAngularPosition()
{
    return mParentMobility->getCurrentAngularPosition() + mOffsetAngles;
}

// TODO conversion will cause near zero values
inet::Quaternion AntennaMobility::getCurrentAngularVelocity()
{
    inet::Quaternion speed = mParentMobility->getCurrentAngularVelocity();
    inet::EulerAngles speedAngles = speed.toEulerAngles();
    if (speedAngles.alpha != inet::rad(0.0) || speedAngles.beta != inet::rad(0.0) || speedAngles.gamma != inet::rad(0.0)) {
        throw omnetpp::cRuntimeError("non-zero angular velocity is not supported");
    }

    return inet::Quaternion::IDENTITY;
}

// TODO conversion will cause near zero values
inet::Quaternion AntennaMobility::getCurrentAngularAcceleration()
{
    inet::Quaternion speed = mParentMobility->getCurrentAngularAcceleration();
    inet::EulerAngles speedAngles = speed.toEulerAngles();
    if (speedAngles.alpha != inet::rad(0.0) || speedAngles.beta != inet::rad(0.0) || speedAngles.gamma != inet::rad(0.0)) {
        throw omnetpp::cRuntimeError("non-zero angular acceleration is not supported");
    }

    return inet::Quaternion::IDENTITY;
}

inet::Coord AntennaMobility::getConstraintAreaMax() const
{
    inet::Coord offset = mParentMobility->getConstraintAreaMax() + mOffsetCoord;
    return offset.max(mParentMobility->getConstraintAreaMax());
}

inet::Coord AntennaMobility::getConstraintAreaMin() const
{
    inet::Coord offset = mParentMobility->getConstraintAreaMin() + mOffsetCoord;
    return offset.min(mParentMobility->getConstraintAreaMin());
}

} // namespace artery
