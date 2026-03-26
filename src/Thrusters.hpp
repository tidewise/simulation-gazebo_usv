#ifndef _GAZEBO_USV_THRUSTERS_HPP_
#define _GAZEBO_USV_THRUSTERS_HPP_

#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport.hh>
#include <sdformat.hh>

#include <gz/gazebo_usv/thrusters.pb.h>
#include <gazebo_usv/Thruster.hpp>

namespace gazebo_usv {
    /** Management of all the thrusters in a given model */
    class Thrusters {
        typedef gz::math::Vector3d Vector3d;

    public:
        Thrusters() = default;
        Thrusters(Thrusters const&) = delete;
        ~Thrusters();

        /**
         * @param plugin_sdf the SDF element of the <plugin ...> tag that contains
         *    thrusters
         */
        void load(std::shared_ptr<gz::transport::Node> node,
            gz::sim::Entity model,
            sdf::ElementConstPtr plugin_sdf,
            gz::sim::EntityComponentManager& ecm);
        void update(gz::sim::EntityComponentManager& ecm);

        Thruster& getThrusterByName(std::string const& name);

    private:
        void processThrusterCommand(gz::gazebo_usv::Thrusters const& thrusters_msg);

        std::vector<Thruster> m_definitions;

        gz::sim::Entity m_model;

        /**
         * @param plugin_sdf the SDF element of the <plugin ...> tag that contains
         *    thrusters
         */
        std::vector<Thruster> loadThrusters(gz::sim::Entity model,
            sdf::ElementConstPtr plugin_sdf,
            gz::sim::EntityComponentManager& ecm

        );

        /** Apply the min/max thrust to thruster effort
         */
        void clampThrustEffort(Thruster& thruster);
    };
}

#endif
