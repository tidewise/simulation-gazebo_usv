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

namespace gazebo_usv {
    class USVPlugin : public gazebo::ModelPlugin {
    public:
        ~USVPlugin();
        virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
        Rudder& getRudderByName(std::string const& name);
        Thruster& getThrusterByName(std::string const& name);

    private:
        gazebo::event::ConnectionPtr mWorldUpdateEvent;
        gazebo::transport::NodePtr mNode;
        gazebo::physics::ModelPtr mModel;

        Actuators* mActuators = nullptr;
        std::vector<Rudder> mRudders;
        Thrusters* mThrusters = nullptr;
        Wind* mWind = nullptr;

        void updateBegin(gazebo::common::UpdateInfo const& info);

        std::vector<Rudder> loadRudders(sdf::ElementPtr pluginElement);
        Thrusters* loadThrusters(sdf::ElementPtr pluginElement);
        Wind* loadWindParameters(sdf::ElementPtr pluginElement);
    };
}

#endif