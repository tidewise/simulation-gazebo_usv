#ifndef GAZEBO_WAVE_PLUGIN_HPP
#define GAZEBO_WAVE_PLUGIN_HPP

#include "base/Time.hpp"
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo_usv {
    class Wave {
        typedef gazebo::physics::ModelPtr ModelPtr;
        typedef gazebo::physics::LinkPtr LinkPtr;
        typedef gazebo::transport::NodePtr NodePtr;
        typedef gazebo::transport::SubscriberPtr SubscriberPtr;

    public:
        // Wave force and torque
        struct Effects {
            ignition::math::Vector3d force = ignition::math::Vector3d::Zero;
            ignition::math::Vector3d torque = ignition::math::Vector3d::Zero;
        };
        // Wave phases to be defined
        double m_phase_x = 0;
        double m_phase_y = 0;
        double m_phase_z = 0;
        double m_phase_n = 0;

        Wave() = default;
        ~Wave();

        /**
         * @brief Loads the model properties from the SDF file and the gazebo
         * communication node.
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
         *  - Fossen's Handbook of Marine Craft Hydrodynamics and Motion Control: pages
         * 188 to 192.
         *
         * PS: This method is public for testing purposes and shouldn't be used for other
         * reasons.
         * @param body2world_orientation vessel orientation in world frame
         * @param vessel_linear_vel_world vessel linear amplitude in world frame
         * @param wave_amplitude_world wave amplitude in world frame
         * @return Effects resulting force and torque to be applied at the vessel CoG.
         */
        Effects computeEffects(double seconds,
            ignition::math::Vector3d const wave_amplitude_world,
            ignition::math::Vector3d const wave_frequency_world,
            double const roll_amplitude_world,
            double const roll_frequency_world) const;

    private:
        ModelPtr m_model;
        NodePtr m_node;
        LinkPtr m_link;

        SubscriberPtr m_wave_amplitude_subscriber;
        SubscriberPtr m_wave_frequency_subscriber;
        SubscriberPtr m_roll_subscriber;

        ignition::math::Vector3d m_wave_amplitude = ignition::math::Vector3d::Zero;
        ignition::math::Vector3d m_wave_frequency = ignition::math::Vector3d::Zero;
        double m_roll_amplitude = 0;
        double m_roll_frequency = 0;

        /**
         * @brief Get the reference link where force and torque will be applied
         *
         * @param model model pointer
         * @param sdf sdf element
         * @return LinkPtr reference link.
         */
        LinkPtr getReferenceLink(ModelPtr const model, sdf::ElementPtr const sdf) const;


        /**
         * @brief Subscriber callback for the wave amplitude topic
         *
         */
        void readWaveAmplitude(const ConstVector3dPtr&);
        void readWaveFrequency(const ConstVector3dPtr&);
        void readRoll(const ConstVector2dPtr&);
    };
}

#endif