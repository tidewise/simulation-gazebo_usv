#ifndef GAZEBO_THRUSTER_PLUGIN_HPP
#define GAZEBO_THRUSTER_PLUGIN_HPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo_usv/Actuators.hpp>
#include <gazebo_usv/Rudder.hpp>
#include <gazebo_usv/Thrusters.hpp>
#include <gazebo_usv/Thruster.hpp>
#include <gazebo_usv/Wind.hpp>
#include <gazebo_usv/Wave.hpp>
#include <gazebo_usv/DirectForceApplication.hpp>

namespace gazebo_usv {
    class USVPlugin : public gazebo::ModelPlugin {
    public:
        ~USVPlugin();
        virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _plugin_sdf);
        Rudder& getRudderByName(std::string const& name);
        Thruster& getThrusterByName(std::string const& name);

    private:
        gazebo::event::ConnectionPtr m_world_update_event;
        gazebo::transport::NodePtr m_node;
        gazebo::physics::ModelPtr m_model;

        Actuators* m_actuators = nullptr;
        std::vector<Rudder> m_rudders;
        Thrusters* m_thrusters = nullptr;
        Wind* m_wind = nullptr;
        Wave* m_wave = nullptr;
        DirectForceApplication* m_direct_force = nullptr;

        void updateBegin(gazebo::common::UpdateInfo const& info);

        std::vector<Rudder> loadRudders(sdf::ElementPtr plugin_element);
        Thrusters* loadThrusters(sdf::ElementPtr plugin_element);
        Wind* loadWindParameters(sdf::ElementPtr plugin_element);
        Wave* loadWaveParameters(sdf::ElementPtr plugin_element);
        DirectForceApplication* loadDirectForceApplicationParameters(sdf::ElementPtr plugin_element);
    };
}

#endif