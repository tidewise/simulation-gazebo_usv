#include <gazebo_usv/Rudder.hpp>
#include "Utilities.hpp"
#include "Actuators.hpp"

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;
using namespace ignition::math;

Rudder::Rudder(Actuators& actuators, physics::ModelPtr model, sdf::ElementPtr sdf) {
    mLinkName = sdf->Get<string>("name");
    mLink = model->GetLink(mLinkName);
    if (!mLink) {
        gzthrow("Rudder: link " + mLinkName + " does not exist");
    }

    mActuatorID = actuators.addLink(mLink);

    mArea = sdf->Get<float>("area", 1).first;
    mLiftK = sdf->Get<float>("lift_factor", 1.5).first;
    mDragK = sdf->Get<float>("drag_factor", 1e-3).first;
}

Rudder::~Rudder() {
}

void Rudder::update(Actuators& actuators) {
    // get linear velocity at cp in inertial frame
    auto vel = mLink->WorldLinearVel(); // - waterCurrent;
    if (vel.Length () <= 0.01) {
        return;
    }

    // pose of body
    auto pose = mLink->WorldPose();

    // rotate forward and upward vectors into inertial frame
    auto forwardI = pose.Rot().RotateVector(Vector3d::UnitX);
    auto upwardI = pose.Rot().RotateVector(Vector3d::UnitZ);

    // ldNormal vector to lift-drag-plane described in inertial frame
    auto ldNormal = forwardI.Cross(upwardI).Normalize();

    // angle of attack
    auto velInLDPlane = ldNormal.Cross(vel.Cross(ldNormal));

    // get direction of drag
    auto dragDirection = -velInLDPlane.Normalized();

    // get direction of lift
    auto liftDirection = ldNormal.Cross(velInLDPlane).Normalized();

    float alpha = atan2(-upwardI.Dot(velInLDPlane), forwardI.Dot(velInLDPlane));

    // compute dynamic pressure
    double speedInLDPlane = velInLDPlane.Length();
    double q = 0.5 * mFluidDensity * speedInLDPlane * speedInLDPlane;

    // compute cl at cp, check for stall, correct for sweep
    double cl = mLiftK * sin(2*alpha);
    // compute lift force at cp
    auto lift = cl * q * mArea * liftDirection;

    // compute cd at cp, check for stall, correct for sweep
    double cd = fabs(mDragK * (1 - cos(2 * alpha)));

    // drag at cp
    auto drag = cd * q * mArea * dragDirection;

    Vector3d worldForce = lift + drag;
    Vector3d linkForce = pose.Rot().RotateVectorReverse(worldForce);
    actuators.applyForce(mActuatorID, linkForce);
}
