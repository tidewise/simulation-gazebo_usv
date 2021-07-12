#ifndef _GAZEBOTHRUSTER_HPP_
#define _GAZEBOTHRUSTER_HPP_

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_underwater/DataTypes.hpp>

#include "msgs.pb.h"

namespace gazebo_usv {
    class Actuators;
    class USVPlugin;
    class Rudder;

    /** Management of all the thrusters in a given model */
    class Thrusters {
    public:
        Thrusters() = default;
        Thrusters(Thrusters const&) = delete;
        ~Thrusters();

        void load(
            USVPlugin& plugin,
            Actuators& actuators, gazebo::transport::NodePtr node,
            gazebo::physics::ModelPtr model, sdf::ElementPtr pluginElement
        );
        void update(Actuators& actuators);

    private:
        typedef const boost::shared_ptr<const gazebo_thruster::msgs::Thrusters>
            ThrustersMSG;

        void processThrusterCommand(ThrustersMSG const& thrustersMSG);

        struct Definition {
            std::string name;
            size_t actuatorID;
            gazebo::physics::LinkPtr link;
            double minThrust;
            double maxThrust;
            double effort;

            Rudder* associatedRudder = nullptr;
        };

        std::vector<Definition> mDefinitions;

        gazebo::physics::ModelPtr mModel;
        gazebo::transport::SubscriberPtr mCommandSubscriber;

        std::vector<Definition> loadThrusters(
            USVPlugin& plugin, Actuators& actuators, sdf::ElementPtr pluginElement
        );

        /** Apply the min/max thrust to thruster effort
         */
        void clampThrustEffort(Definition& thruster);
    };
}

#endif
