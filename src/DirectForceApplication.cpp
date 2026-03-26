#include <gazebo_usv/DirectForceApplication.hpp>

#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Link.hh>
#include <stdexcept>
#include <string>

#include "RockGazeboHelpers.hpp"

using namespace gazebo_usv;

DirectForceApplication::~DirectForceApplication() {
}

void DirectForceApplication::load(
    gz::sim::Entity model,
    std::shared_ptr<gz::transport::Node> node,
    sdf::ElementConstPtr plugin_sdf,
    gz::sim::EntityComponentManager& ecm
) {
    auto link_name = plugin_sdf->Get<std::string>("link");
    if (link_name.empty()) {
        std::string msg = "DirectForceApplication: sdf model loads 'gazebo_usv_force' plugin,\n"
                      "but does not defines a link parameter. Please name the link\n"
                      "you want to apply a force inside the <plugin> tag, e.g.:\n"
                      "<link>'link_1'</link> ";
        throw std::invalid_argument(msg);
    }

    m_link = rock_gazebo_helpers::resolveLinkRecursive(model, link_name, ecm);
    gzmsg << "DirectForceApplication: applying on link " << gz::sim::scopedName(model, ecm, "::", false) << std::endl;

    // Initialize communication node and subscribe to gazebo topic
    std::string topic_name = gz::sim::topicFromScopedName(m_link, ecm) + "/gazebo_usv_force";
    node->Subscribe(topic_name, &DirectForceApplication::processDirectionalForceCommand, this);

    gzmsg << "DirectForceApplication: receiving direct force commands from "
          << topic_name << std::endl;
}

void DirectForceApplication::processDirectionalForceCommand(gz::msgs::Vector3d const& force_msg) {
    m_force_cmd = rock_gazebo_helpers::proto2Gz(force_msg);
}

void DirectForceApplication::update(gz::sim::EntityComponentManager& ecm) {
    gz::sim::Link(m_link).AddForceInInertialFrame(ecm, m_force_cmd);
}

