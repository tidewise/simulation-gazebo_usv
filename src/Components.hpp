#ifndef GAZEBO_USV_COMPONENTS_HPP
#define GAZEBO_USV_COMPONENTS_HPP

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>
#include <string>

namespace gazebo_usv {
    /// \brief Name of the transport topic where a wave effect plugin is expecting the
    /// wave parameters
    using WaveParametersTopic = gz::sim::components::Component<std::string,
        class WaveParametersTopicTag,
        gz::sim::serializers::StringSerializer>;
    GZ_SIM_REGISTER_COMPONENT("gazebo_usv_components.WaveParametersTopic", WaveParametersTopic)

    /// \brief Name of the transport topic where a wind speed plugin is expecting the
    /// wind speed.
    using WindSpeedTopic = gz::sim::components::Component<std::string,
        class WindSpeedTag,
        gz::sim::serializers::StringSerializer>;
    GZ_SIM_REGISTER_COMPONENT("gazebo_usv_components.WindSpeedTopic", WindSpeedTopic)

    /// \brief Name of the transport topic where a thruster is expecting its
    /// command.
    ///
    /// When the thruster's topic is explicitly given, this component contains the
    /// basename of the actual topic
    using ThrustersTopic = gz::sim::components::Component<std::string,
        class ThrustersTopicTag,
        gz::sim::serializers::StringSerializer>;
        GZ_SIM_REGISTER_COMPONENT("gazebo_usv_components.ThrustersTopic", ThrustersTopic)

    /// \brief Water level used to compute buoyancy for a link.
    using WaterLevel = gz::sim::components::Component<double,
        class WaterLevelTag,
        gz::sim::serializers::DefaultSerializer<double>>;
    GZ_SIM_REGISTER_COMPONENT("gazebo_usv_components.WaterLevel", WaterLevel)
}

#endif
