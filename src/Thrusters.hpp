#ifndef _GAZEBO_USV_THRUSTERS_HPP_
#define _GAZEBO_USV_THRUSTERS_HPP_

#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo_underwater/DataTypes.hpp>

#include "msgs.pb.h"
#include <gazebo_usv/Thruster.hpp>

namespace gazebo_usv {
    class Actuators;

    /** Management of all the thrusters in a given model */
    class Thrusters {
        typedef ignition::math::Vector3d Vector3d;
    public:
        Thrusters() = default;
        Thrusters(Thrusters const&) = delete;
        ~Thrusters();

        void load(
            Actuators& actuators, gazebo::transport::NodePtr node,
            gazebo::physics::ModelPtr model, sdf::ElementPtr pluginElement
        );
        void update(Actuators& actuators);

        Thruster& getThrusterByName(std::string const& name);

    private:
        typedef const boost::shared_ptr<const gazebo_thruster::msgs::Thrusters>
            ThrustersMSG;

        void processThrusterCommand(ThrustersMSG const& thrustersMSG);

        std::vector<Thruster> mDefinitions;

        gazebo::physics::ModelPtr mModel;
        gazebo::transport::SubscriberPtr mCommandSubscriber;

        std::vector<Thruster> loadThrusters(
            Actuators& actuators, sdf::ElementPtr pluginElement
        );

        /** Apply the min/max thrust to thruster effort
         */
        void clampThrustEffort(Thruster& thruster);
    };
}

#endif
