#include "Thruster.hpp"
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>

using namespace gazebo_usv;
using namespace gz::sim;

std::string Thruster::getLinkName() const {
    return name;
}

gz::math::Vector3d Thruster::getWorldLinearVelocity(gz::sim::EntityComponentManager& ecm) const {
    return Link(link).WorldLinearVelocity(ecm).value();

}
gz::math::Pose3d Thruster::getWorldPose(gz::sim::EntityComponentManager& ecm) const {
    return Link(link).WorldPose(ecm).value();
}

float Thruster::getEffort() const {
    return effort;
}

float Thruster::getAdvanceSpeed(gz::sim::EntityComponentManager& ecm) const {
    auto vel = Link(link).WorldLinearVelocity(ecm);
    auto pose = Link(link).WorldPose(ecm);
    if (!vel.has_value() || !pose.has_value()) {
        return 0;
    }

    auto forward_i = pose.value().Rot().RotateVector(Vector3d::UnitX);
    return forward_i.Dot(vel.value());
}
