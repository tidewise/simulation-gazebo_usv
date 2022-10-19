#ifndef GAZEBO_USV_DIRECT_FORCEAPPLICATION_HPP
#define GAZEBO_USV_DIRECT_FORCEAPPLICATION_HPP

#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/msgs/vector3d.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <sdf/Element.hh>

#include <ignition/math/Vector3.hh>

namespace gazebo_usv {
    class Actuators;
    /**
     * @brief Gazebo model plugin for applying directed force on a link
     * 
     */
    class DirectForceApplication {
    public:
        ~DirectForceApplication();
        void load(
            Actuators& actuators, gazebo::physics::ModelPtr model,
            gazebo::transport::NodePtr node, sdf::ElementPtr plugin_element
        );
        void update(Actuators& actuators);

    private:
        gazebo::transport::SubscriberPtr m_command_subscriber;
        ignition::math::Vector3d m_force_cmd;
        size_t m_link_id;

        void processDirectionalForceCommand(ConstVector3dPtr const& force_msg);
    };
}

#endif
