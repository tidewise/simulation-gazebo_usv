#ifndef GAZEBO_USV_THRUSTER_HPP
#define GAZEBO_USV_THRUSTER_HPP

#include <gazebo/physics/physics.hh>

namespace gazebo_usv {
    class Thruster {
        friend class Thrusters;
        typedef ignition::math::Vector3d Vector3d;

    public:

        std::string getLinkName() const;
        gazebo::physics::LinkPtr getLink();
        float getEffort() const;
        float getAdvanceSpeed() const;

    private:
        std::string name;
        size_t actuator_id;
        gazebo::physics::LinkPtr link;
        double min_thrust = 0;
        double max_thrust = 0;
        double effort = 0;
    };
}

#endif