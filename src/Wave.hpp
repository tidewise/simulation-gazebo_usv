#ifndef GAZEBO_WAVE_PLUGIN_HPP
#define GAZEBO_WAVE_PLUGIN_HPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <time.h>

namespace gazebo_usv
{
    class Wave
    {
        typedef gazebo::physics::ModelPtr ModelPtr;
        typedef gazebo::physics::LinkPtr LinkPtr;
        typedef gazebo::transport::NodePtr NodePtr;
        typedef gazebo::transport::SubscriberPtr SubscriberPtr;

    public:
        // Parameters used to calculate the Wave effects 
        struct EffectParameters
        {
            double frontal_area;
            double lateral_area;
            double bottom_area;
            double torque_constant;
            double length_overall;
            double water_density;
            ignition::math::Vector3d coefficients;
        };
        // Wave force and torque
        struct Effects
        {
            ignition::math::Vector3d force;
            ignition::math::Vector3d torque;
        };

        Wave() = default;
        /**
         * @brief Construct a new Wave object.
         * 
         * PS: This constructor is used mainly for easier testing.
         * 
         * @param parameters 
         */
        Wave(EffectParameters const parameters);
        ~Wave();

        /**
         * @brief Loads the model properties from the SDF file and the gazebo communication node.
         * 
         * @param _model model pointer
         * @param _node gazebo node
         * @param _sdf sdf element
         */
        void load(ModelPtr const _model, NodePtr const _node, sdf::ElementPtr const _sdf);

        /**
         * @brief Update the wave effects on the model.
         * 
         */
        void update();

        /**
         * @brief Computes the wave effects on a vessel. 
         * 
         * Reference: 
         *  - Fossen's Handbook of Marine Craft Hydrodynamics and Motion Control: pages 188 to 192.
         * 
         * PS: This method is public for testing purposes and shouldn't be used for other reasons.
         * @param body2world_orientation vessel orientation in world frame 
         * @param vessel_linear_vel_world vessel linear amplitude in world frame 
         * @param wave_amplitude_world wave amplitude in world frame 
         * @return Effects resulting force and torque to be applied at the vessel CoG. 
         */
        Effects computeEffects(ignition::math::Quaterniond const body2world_orientation, ignition::math::Vector3d const vessel_linear_vel_world, ignition::math::Vector3d const wave_amplitude_world, ignition::math::Vector3d const wave_frequency_world) const;

    private:
        ModelPtr mModel;
        NodePtr mNode;
        LinkPtr mLink;

        double phase_x;
        double phase_y;
        double phase_z;
        double phase_n;

        SubscriberPtr mWaveAmplitudeSubscriber;
        SubscriberPtr mWaveFrequencySubscriber;

        EffectParameters mParameters;
        ignition::math::Vector3d mWaveAmplitude{};
        ignition::math::Vector3d mWaveFrequency{};

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
         * @brief Subscriber callback for the wave amplitude topic
         * 
         */
        void readWaveAmplitude(const ConstVector3dPtr &);
        void readWaveFrequency(const ConstVector3dPtr &);
    };
}

#endif