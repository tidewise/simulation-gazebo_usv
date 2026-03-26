#ifndef GAZEBO_WIND_PLUGIN_HPP
#define GAZEBO_WIND_PLUGIN_HPP

#include <gz/math/Vector3.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/transport.hh>
#include <sdf/Element.hh>

namespace gazebo_usv {
    class Wind {
    public:
        // Parameters used to calculate the wind effects
        struct EffectParameters {
            double frontal_area = 0;
            double lateral_area = 0;
            double length_overall = 0;
            double air_density = 0;
            gz::math::Vector3d coefficients = gz::math::Vector3d::Zero;
        };
        // Wind force and torque
        struct Effects {
            gz::math::Vector3d force = gz::math::Vector3d::Zero;
            gz::math::Vector3d torque = gz::math::Vector3d::Zero;
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
         * @brief Loads the model properties from the SDF file and the gazebo
         * communication node.
         *
         * @param model
         * @param node
         * @param plugin_sdf SDF `plugin` element
         */
        void load(gz::sim::Entity model,
            std::shared_ptr<gz::transport::Node> node,
            sdf::ElementConstPtr const plugin_sdf,
            gz::sim::EntityComponentManager& ecm);

        /**
         * @brief Update the wind effects on the model.
         *
         */
        void update(gz::sim::EntityComponentManager& ecm);

        /**
         * @brief Computes the wind effects on a vessel.
         *
         * Reference:
         *  - Fossen's Handbook of Marine Craft Hydrodynamics and Motion Control: pages
         * 188 to 192.
         *
         * PS: This method is public for testing purposes and shouldn't be used for other
         * reasons.
         * @param body2world_orientation vessel orientation in world frame
         * @param vessel_linear_vel_world vessel linear velocity in world frame
         * @param wind_velocity_world wind velocity in world frame
         * @return Effects resulting force and torque to be applied at the vessel CoG.
         */
        Effects computeEffects(gz::math::Quaterniond const body2world_orientation,
            gz::math::Vector3d const vessel_linear_vel_world,
            gz::math::Vector3d const wind_velocity_world) const;

    private:
        gz::sim::Entity m_model;
        std::shared_ptr<gz::transport::Node> m_node;
        gz::sim::Entity m_link;
        std::string m_topic_name;

        EffectParameters m_parameters;
        gz::math::Vector3d m_wind_velocity = gz::math::Vector3d::Zero;

        /**
         * @brief Load parameters from SDF file
         *
         * @param el SDF element
         * @return EffectParameters loaded parameters
         */
        EffectParameters loadParameters(sdf::ElementConstPtr const el) const;

        /**
         * @brief Subscriber callback for the wind velocity topic
         *
         */
        void readWindVelocity(gz::msgs::Vector3d const&);
    };
}

#endif