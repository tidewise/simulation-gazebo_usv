#ifndef GAZEBO_THRUSTER_RUDDER_HPP
#define GAZEBO_THRUSTER_RUDDER_HPP

#include <gz/sim/EntityComponentManager.hh>
#include <sdf/Element.hh>
#include <gz/sim/Entity.hh>
#include <gz/transport.hh>

namespace gazebo_usv {
    class USVPlugin;
    class Thruster;

    class Rudder {
        typedef gz::math::Vector3d Vector3d;

    public:
        /**
         * @param sdf the SDF element that describes the rudder within the plugin.
         *   The <plugin ...> tag is expected to be its direct parent
         */
        Rudder(USVPlugin& plugin, gz::sim::Entity model,
               sdf::ElementConstPtr rudder_sdf, gz::sim::EntityComponentManager& ecm);
        ~Rudder();

        void update(gz::sim::EntityComponentManager& ecm);
        std::string getLinkName() const;
        Vector3d getFlowVelocity(gz::sim::EntityComponentManager& ecm) const;

    private:
        std::string m_link_name;
        gz::sim::Entity m_link;
        Thruster* m_associated_thruster = nullptr;

        float m_fluid_density = 1000;
        float m_area = 1;
        float m_lift_k = 1.5;
        float m_drag_k = 1;
        float m_thrust_to_speed_k = 0;
    };
}

#endif
