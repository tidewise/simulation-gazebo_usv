#ifndef GAZEBO_WIND_PLUGIN_HPP
#define GAZEBO_WIND_PLUGIN_HPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo_usv
{
    class Wind
    {
        typedef gazebo::physics::ModelPtr ModelPtr;
        typedef gazebo::physics::LinkPtr LinkPtr;
        typedef gazebo::transport::NodePtr NodePtr;
        typedef gazebo::transport::SubscriberPtr SubscriberPtr;


    public:
        Wind() = default;
        ~Wind();

        void load(ModelPtr const _model, NodePtr const _node, sdf::ElementPtr const _sdf);
        void update();

    private:
        // Parameters used to calculate the wind effects read from the SDF
        struct EffectParameters
        {
            double frontal_area;
            double lateral_area;
            double length_overall;
            double air_density;
            ignition::math::Vector3d coefficients;
        };
        // Wind force and torque 
        struct Effects
        {
            ignition::math::Vector3d force;
            ignition::math::Vector3d torque;
        };

        ModelPtr mModel;
        NodePtr mNode;
        LinkPtr mLink;

        SubscriberPtr mWindVelocitySubscriber;

        EffectParameters mParameters;
        ignition::math::Vector3d mWindVelocity;

        // Compute wind effects
        Effects computeEffects();

        // Get link where the wind effects will be applied
        LinkPtr getReferenceLink(ModelPtr const model, sdf::ElementPtr const sdf) const;
        EffectParameters loadParameters(sdf::ElementPtr const el) const;

        // Read from the wind velocity topic
        void readWindVelocity(const ConstVector3dPtr &);
    };
}

#endif