#ifndef GAZEBO_USV_BUOYANCY_HPP
#define GAZEBO_USV_BUOYANCY_HPP

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport.hh>
#include <sdf/Element.hh>

namespace gazebo_usv {
    class USVPlugin;

    class Buoyancy {
        std::string m_link_name;
        gz::sim::Entity m_link;

        /** Buoyancy force when the whole body is submerged, in newtons */
        double m_buoyancy_force = 0;

        /* Water level used by the plugin if a specific water level is not
         * stored on the link via the WaterLevel component
         */
        double m_default_water_level = 0;

        /* Offset between the center of gravity and the center of buoyancy, in the
         * link-fixed frame
         */
        gz::math::Vector3d m_center_of_buoyancy = gz::math::Vector3d::Zero;

        std::optional<double> calculateSubmersedRatio(
            gz::sim::EntityComponentManager& ecm) const;

    public:
        Buoyancy(USVPlugin& plugin,
            gz::sim::Entity model,
            sdf::ElementConstPtr buoyancy_sdf,
            gz::sim::EntityComponentManager& ecm,
            double water_level);

        void update(gz::sim::EntityComponentManager& ecm);
    };
}

#endif
