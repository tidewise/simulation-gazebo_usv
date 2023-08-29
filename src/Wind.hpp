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
        // Parameters used to calculate the wind effects 
        struct EffectParameters
        {
            double frontal_area = 0;
            double lateral_area = 0;
            double length_overall = 0;
            double air_density = 0;
            ignition::math::Vector3d coefficients = ignition::math::Vector3d::Zero;
        };
        // Wind force and torque
        struct Effects
        {
            ignition::math::Vector3d force = ignition::math::Vector3d::Zero;
            ignition::math::Vector3d torque = ignition::math::Vector3d::Zero;
        };

        Wind() = default;
        /**
         * @brief Construct a new Wind object.
         * 
         * PS: This constructor is used mainly for easier testing.
         * 
         * @param parameters 
         */
        Wind(EffectParameters const parameters);
        ~Wind();

        /**
         * @brief Loads the model properties from the SDF file and the gazebo communication node.
         * 
         * @param _model model pointer
         * @param _node gazebo node
         * @param _sdf sdf element
         */
        void load(ModelPtr const _model, NodePtr const _node, sdf::ElementPtr const _sdf);

        /**
         * @brief Update the wind effects on the model.
         * 
         */
        void update();

        /**
         * @brief Computes the wind effects on a vessel. 
         * 
         * Reference: 
         *  - Fossen's Handbook of Marine Craft Hydrodynamics and Motion Control: pages 188 to 192.
         * 
         * PS: This method is public for testing purposes and shouldn't be used for other reasons.
         * @param body2world_orientation vessel orientation in world frame 
         * @param vessel_linear_vel_world vessel linear velocity in world frame 
         * @param wind_velocity_world wind velocity in world frame 
         * @return Effects resulting force and torque to be applied at the vessel CoG. 
         */
        Effects computeEffects(ignition::math::Quaterniond const body2world_orientation, ignition::math::Vector3d const vessel_linear_vel_world, ignition::math::Vector3d const wind_velocity_world) const;

    private:
        ModelPtr m_model;
        NodePtr m_node;
        LinkPtr m_link;

        SubscriberPtr m_wind_velocity_subscriber;

        EffectParameters m_parameters;
        ignition::math::Vector3d m_wind_velocity = ignition::math::Vector3d::Zero;

        /**
         * @brief Get the reference link where force and torque will be applied
         * 
         * @param model model pointer
         * @param sdf sdf element
         * @return LinkPtr reference link.
         */
        LinkPtr getReferenceLink(ModelPtr const model, sdf::ElementPtr const sdf) const;

        /**
         * @brief Load parameters from SDF file
         * 
         * @param el SDF element
         * @return EffectParameters loaded parameters
         */
        EffectParameters loadParameters(sdf::ElementPtr const el) const;

        /**
         * @brief Subscriber callback for the wind velocity topic
         * 
         */
        void readWindVelocity(const ConstVector3dPtr &);
    };
}

#endif