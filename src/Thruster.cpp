#include "Thruster.hpp"

using namespace gazebo_usv;
using namespace ignition::math;

std::string Thruster::getLinkName() const {
    return name;
}

gazebo::physics::LinkPtr Thruster::getLink() {
    return link;
}


float Thruster::getEffort() const {
    return effort;
}

float Thruster::getAdvanceSpeed() const {
    auto vel = link->WorldLinearVel();
    auto pose = link->WorldPose();
    auto forwardI = pose.Rot().RotateVector(Vector3d::UnitX);
    return forwardI.Dot(vel);
}
