#ifndef GAZEBO_THRUSTER_RUDDER_HPP
#define GAZEBO_THRUSTER_RUDDER_HPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_underwater/DataTypes.hpp>

namespace gazebo_usv {
    class USVPlugin;
    class Actuators;
    class Thruster;

    class Rudder {
        typedef ignition::math::Vector3d Vector3d;

    public:
        Rudder(USVPlugin& plugin, Actuators& actuators, gazebo::physics::ModelPtr model,
               sdf::ElementPtr sdf);
        ~Rudder();

        void update(Actuators& actuator);
        std::string getLinkName() const;
        Vector3d getFlowVelocity() const;

    private:
        std::string m_link_name;
        gazebo::physics::LinkPtr m_link;
        Thruster* m_associated_thruster = nullptr;
        size_t m_actuator_id;

        float m_fluid_density = 1000;
        float m_area = 1;
        float m_lift_k = 1.5;
        float m_drag_k = 1;
        float m_thrust_to_speed_k = 0;
    };
}

#endif
