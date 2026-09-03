#ifndef GAZEBO_THRUSTER_PLUGIN_HPP
#define GAZEBO_THRUSTER_PLUGIN_HPP

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport.hh>

#include <gazebo_usv/DirectForceApplication.hpp>
#include <gazebo_usv/Buoyancy.hpp>
#include <gazebo_usv/Rudder.hpp>
#include <gazebo_usv/Thruster.hpp>
#include <gazebo_usv/Thrusters.hpp>
#include <gazebo_usv/Wave.hpp>
#include <gazebo_usv/Wind.hpp>

namespace gazebo_usv {
    class USVPlugin : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate {
    public:
        ~USVPlugin();

        Rudder& getRudderByName(std::string const& name);
        Thruster& getThrusterByName(std::string const& name);

        /** Method called during the Configure step of the gazebo lifecycle
         *
         * @param entity the entity the <plugin> tag is attached to
         * @param sdf the SDF element representing the <plugin ...> tag for
         *    this plugin. It has no parent (so, can't discover the SDF definition
         *    of the entity)
         */
        void Configure(gz::sim::Entity const& entity,
            std::shared_ptr<const sdf::Element> const& plugin_sdf,
            gz::sim::EntityComponentManager& ecm,
            gz::sim::EventManager& event_manager) override;

        /** Method called during the PreUpdate step of the gazebo lifecycle
         */
        void PreUpdate(gz::sim::UpdateInfo const& info,
            gz::sim::EntityComponentManager& ecm) override;

    private:
        std::shared_ptr<gz::transport::Node> m_node;
        gz::sim::Entity m_model;

        std::vector<Rudder> m_rudders;
        std::vector<Buoyancy> m_buoyancy;
        Thrusters* m_thrusters = nullptr;
        Wind* m_wind = nullptr;
        Wave* m_wave = nullptr;
        DirectForceApplication* m_direct_force = nullptr;

        std::vector<Rudder> loadRudders(sdf::ElementConstPtr plugin_sdf,
            gz::sim::EntityComponentManager& ecm);
        std::vector<Buoyancy> loadBuoyancy(sdf::ElementConstPtr plugin_sdf,
            gz::sim::EntityComponentManager& ecm);
        Thrusters* loadThrusters(sdf::ElementConstPtr plugin_sdf,
            gz::sim::EntityComponentManager& ecm);
        Wind* loadWindParameters(sdf::ElementConstPtr plugin_sdf,
            gz::sim::EntityComponentManager& ecm);
        Wave* loadWaveParameters(sdf::ElementConstPtr plugin_sdf,
            gz::sim::EntityComponentManager& ecm);
        DirectForceApplication* loadDirectForceApplicationParameters(
            sdf::ElementConstPtr plugin_sdf,
            gz::sim::EntityComponentManager& ecm);
    };
}

#endif
