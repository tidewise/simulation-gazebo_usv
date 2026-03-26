#ifndef GAZEBO_USV_THRUSTER_HPP
#define GAZEBO_USV_THRUSTER_HPP

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

namespace gazebo_usv {
    class Thruster {
        friend class Thrusters;
        typedef gz::math::Vector3d Vector3d;

    public:
        std::string getLinkName() const;
        gz::math::Vector3d getWorldLinearVelocity(gz::sim::EntityComponentManager& ecm) const;
        gz::math::Pose3d getWorldPose(gz::sim::EntityComponentManager& ecm) const;
        float getEffort() const;
        float getAdvanceSpeed(gz::sim::EntityComponentManager& ecm) const;

    private:
        std::string name;
        gz::sim::Entity link;
        double min_thrust = 0;
        double max_thrust = 0;
        double effort = 0;
    };
}

#endif