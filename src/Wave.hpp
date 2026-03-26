#ifndef GAZEBO_WAVE_PLUGIN_HPP
#define GAZEBO_WAVE_PLUGIN_HPP

#include <gz/gazebo_usv/wave.pb.h>

#include <sdf/Element.hh>

#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport.hh>
#include <gz/transport/NodeShared.hh>
#include <sdf/Param.hh>

namespace gazebo_usv {
    class Wave {
    public:
        // Wave force and torque
        struct Effects {
            gz::math::Vector3d force = gz::math::Vector3d::Zero;
            gz::math::Vector3d torque = gz::math::Vector3d::Zero;
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
        void load(gz::sim::Entity model,
            std::shared_ptr<gz::transport::Node> node,
            sdf::ElementConstPtr const sdf,
            gz::sim::EntityComponentManager& ecm);

        /**
         * @brief Update the wave effects on the model.
         *
         */
        void update(gz::sim::EntityComponentManager& ecm);

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
            gz::math::Vector3d const wave_amplitude_world,
            gz::math::Vector3d const wave_frequency_world,
            double const roll_amplitude_world,
            double const roll_frequency_world) const;

    private:
        gz::sim::Entity m_model;
        gz::sim::Entity m_link;
        std::shared_ptr<gz::transport::Node> m_node;
        std::string m_topic_name;

        gz::math::Vector3d m_wave_amplitude = gz::math::Vector3d::Zero;
        gz::math::Vector3d m_wave_frequency = gz::math::Vector3d::Zero;
        double m_roll_amplitude = 0;
        double m_roll_frequency = 0;

        /**
         * @brief Subscriber callback for the wave amplitude topic
         *
         */
        void read(gz::gazebo_usv::Wave const&);
    };
}

#endif