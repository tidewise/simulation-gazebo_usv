#ifndef GAZEBO_USV_COMPONENTS_HPP
#define GAZEBO_USV_COMPONENTS_HPP

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>
#include <string>

namespace gazebo_usv {

    /// \brief Name of the transport topic where a thruster is expecting its
    /// command.
    using ThrustersTopic = gz::sim::components::Component<std::string,
        class ThrustersTopicTag,
        gz::sim::serializers::StringSerializer>;
    GZ_SIM_REGISTER_COMPONENT("gazebo_usv_components.ThrustersTopic", ThrustersTopic)
}

#endif