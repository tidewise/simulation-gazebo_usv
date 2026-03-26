#ifndef GAZEBO_USV_DIRECT_FORCEAPPLICATION_HPP
#define GAZEBO_USV_DIRECT_FORCEAPPLICATION_HPP

#include <gz/msgs/vector3d.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport.hh>

#include <sdf/Element.hh>

#include <gz/math/Vector3.hh>

namespace gazebo_usv {
    class Actuators;
    /**
     * @brief Gazebo model plugin for applying directed force on a link
     *
     */
    class DirectForceApplication {
    public:
        ~DirectForceApplication();

        /**
         * @param plugin_sdf the SDF <plugin ...> element
         */
        void load(
            gz::sim::Entity model,
            std::shared_ptr<gz::transport::Node> node,
            sdf::ElementConstPtr plugin_sdf,
            gz::sim::EntityComponentManager& ecm
        );
        void update(gz::sim::EntityComponentManager& ecm);

    private:
        gz::sim::Entity m_link;
        gz::math::Vector3d m_force_cmd = gz::math::Vector3d::Zero;

        void processDirectionalForceCommand(gz::msgs::Vector3d const& force_msg);
    };
}

#endif
