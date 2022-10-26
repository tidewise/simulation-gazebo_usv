#include <gazebo_usv/DirectForceApplication.hpp>

#include <string>

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <gazebo_usv/Actuators.hpp>
#include "Utilities.hpp"

using namespace gazebo_usv;

DirectForceApplication::~DirectForceApplication() {
    if (m_command_subscriber) {
        m_command_subscriber->Unsubscribe();
    }
}

void DirectForceApplication::load(
    Actuators& actuators, gazebo::physics::ModelPtr model,
    gazebo::transport::NodePtr node, sdf::ElementPtr plugin_element
) {
    auto link_name = plugin_element->Get<std::string>("link");

    if (link_name.empty()) {
        std::string msg = "DirectForceApplication: sdf model loads 'gazebo_usv_force' plugin,\n"
                      "but does not defines a link parameter. Please name the link\n"
                      "you want to apply a force inside the <plugin> tag, e.g.:\n"
                      "<link>'link_1'</link> ";
        gzthrow(msg);
    }

    auto link = utilities::getLinkFromName(model,link_name,plugin_element->Get<std::string>("name"));
    if (!link) {
        std::string msg = "DirectForceApplication: sdf model loads 'direct_force' plugin,\n"
                          "but it's defining an invalid link name, " + link_name + ", as parameter. Please make sure\n"
                          "that the link you're naming actually exists in the model's sdf.";
        gzthrow(msg);
    }

    m_link_id = actuators.addLink(link);

    // Initialize communication node and subscribe to gazebo topic
    auto plugin_name = plugin_element->Get<std::string>("name");
    std::string topic_name = utilities::getNamespaceFromPluginName(plugin_name) + "/" + link->GetName() + "/gazebo_usv_force";
    if (m_command_subscriber) {
        m_command_subscriber->Unsubscribe();
    }
    m_command_subscriber =
        node->Subscribe("/" + topic_name, &DirectForceApplication::processDirectionalForceCommand, this);

    auto world_name = model->GetWorld()->Name();
    gzmsg << "DirectForceApplication: receiving directioned force commands from /"
          << topic_name << std::endl;
}

void DirectForceApplication::processDirectionalForceCommand(ConstVector3dPtr const& force_msg) {
    m_force_cmd.X() = force_msg->has_x() ? force_msg->x() : 0.;
    m_force_cmd.Y() = force_msg->has_y() ? force_msg->y() : 0.;
    m_force_cmd.Z() = force_msg->has_z() ? force_msg->z() : 0.;
}

void DirectForceApplication::update(Actuators& actuators) {
    actuators.applyForce(m_link_id, m_force_cmd);
}

