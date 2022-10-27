#include "Thrusters.hpp"
#include "Utilities.hpp"
#include "Actuators.hpp"
#include "USVPlugin.hpp"

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;
using namespace ignition::math;

Thrusters::~Thrusters() {
    if (m_command_subscriber) {
        m_command_subscriber->Unsubscribe();
    }
}

void Thrusters::load(
    Actuators& actuators, transport::NodePtr node,
    physics::ModelPtr model, sdf::ElementPtr plugin_element
) {
    m_model = model;
    m_definitions = loadThrusters(actuators, plugin_element);

    // Initialize communication node and subscribe to gazebo topic
    auto plugin_name = plugin_element->Get<std::string>("name");
    string topic_name = utilities::getNamespaceFromPluginName(plugin_name) + "/thrusters";

    if (m_command_subscriber) {
        m_command_subscriber->Unsubscribe();
    }
    m_command_subscriber =
        node->Subscribe("/" + topic_name, &Thrusters::processThrusterCommand, this);

    auto world_name = GzGet((*m_model->GetWorld()), Name, ());
    gzmsg << "Thruster: receiving thruster commands from /"
          << topic_name << endl;
}

Thruster& Thrusters::getThrusterByName(std::string const& name) {
    for (auto& thruster : m_definitions) {
        if (thruster.getLinkName() == name) {
            return thruster;
        }
    }
    gzthrow("no thruster with link " + name);
}

std::vector<Thruster> Thrusters::loadThrusters(
    Actuators& actuators, sdf::ElementPtr plugin_element
) {
    std::vector<Thruster> definitions;
    sdf::ElementPtr el = plugin_element->GetElement("thruster");
    while (el) {
        // Load thrusters attributes
        Thruster def;
        def.name = el->Get<string>("name");
        auto link = utilities::getLinkFromName(m_model,def.name,plugin_element->Get<std::string>("name"));
        if (!link) {
            gzthrow("Thruster: thruster " + def.name + " does not exist");
        }

        gzmsg << "Thruster: thruster name: " << def.name << endl;
        def.actuator_id = actuators.addLink(link);
        def.link = link;
        def.min_thrust = utilities::getParameter<double>(
            "Thruster", el, "min_thrust", "N", -200
        );
        def.max_thrust = utilities::getParameter<double>(
            "Thruster", el, "max_thrust", "N", 200
        );
        def.effort = 0.0;
        definitions.push_back(def);
        el = el->GetNextElement("thruster");
    }

    if (definitions.empty())
    {
        string msg = "Thruster: sdf model loads thruster plugin but has no\n"
                     "thruster defined. Please name the links you want to export\n"
                     "as thrusters inside the <plugin> tag, e.g.:\n"
                     "<thruster name='thruster::right'> ";
        gzthrow(msg);
    }
    return definitions;
}

void Thrusters::processThrusterCommand(ThrustersMSG const& thrusters_msg) {
    for (int i = 0; i < thrusters_msg->thrusters_size(); ++i) {
        bool thruster_found = false;
        const gazebo_thruster::msgs::Thruster& thruster_cmd = thrusters_msg->thrusters(i);
        for (auto& thruster : m_definitions) {
            if (thruster_cmd.name() == thruster.name) {
                thruster_found = true;
                thruster.effort = thruster_cmd.has_effort() ? thruster_cmd.effort() : 0;
                clampThrustEffort(thruster);
            }
        }
        if (!thruster_found) {
            gzthrow("Thruster: incoming thruster name: "
                    + thruster_cmd.name() + ", not found.");
        }
    }
}

void Thrusters::clampThrustEffort(Thruster& thruster)
{
    if (thruster.effort < thruster.min_thrust) {
        gzmsg << "Thruster: thruster effort " << thruster.effort
              << " below the minimum\n"
              << "Thruster: using min_thrust: " << thruster.min_thrust
              << " instead" << endl;
        thruster.effort = thruster.min_thrust;
    }
    else if (thruster.effort > thruster.max_thrust)
    {
        gzmsg << "Thruster: thruster effort " << thruster.effort
              << " above the maximum\n"
              << "Thruster: using max_thrust: " << thruster.max_thrust
              << " instead" << endl;
        thruster.effort = thruster.max_thrust;
    }
}

void Thrusters::update(Actuators& actuators) {
    for (auto& thruster : m_definitions) {
        auto thrust = Vector3d::UnitX * thruster.effort;
        actuators.applyForce(thruster.actuator_id, thrust);
    }
}
