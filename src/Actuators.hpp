#ifndef GAZEBO_THRUSTER_ACTUATOR_HPP
#define GAZEBO_THRUSTER_ACTUATOR_HPP

#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_underwater/DataTypes.hpp>

namespace gazebo_usv {
    /**
     * Class that applies added mass corrections on forces before applying them
     * to the underlying model
     */
    class Actuators {
        typedef const boost::shared_ptr<const gazebo_underwater::msgs::CompensatedMass> \
            CompensatedMassMSG;

        struct AddedMassCompensation {
            gazebo::physics::LinkPtr link;
            bool initializedCompensatedEffort = false;
            ignition::math::Vector3d lastModelRelativeDirection;
            ignition::math::Vector3d compensatedDirection;
            ignition::math::Vector3d compensatedPosition;
        };

        gazebo::transport::SubscriberPtr mCompensatedMassSubscriber;

        void readCompensatedMass(CompensatedMassMSG const& msg);
        void updateCompensatedEffort(
            AddedMassCompensation& info,
            ignition::math::Vector3d const& direction,
            gazebo_underwater::Matrix6 const& matrix,
            ignition::math::Vector3d const& cog
        );
        gazebo_underwater::Matrix6 mCurrentMassMatrix;
        ignition::math::Vector3d mCurrentCOG;
        bool mHasCOGAndMassInfo = false;

        std::vector<AddedMassCompensation> mAddedMassCompensation;

    public:
        Actuators(gazebo::physics::ModelPtr model, gazebo::transport::NodePtr node);
        Actuators(Actuators const&) = delete;
        ~Actuators();

        size_t addLink(gazebo::physics::LinkPtr link);
        void applyForce(size_t id, ignition::math::Vector3d const& force);
    };
}

#endif
