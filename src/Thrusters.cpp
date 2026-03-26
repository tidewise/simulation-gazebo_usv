#include "Thrusters.hpp"
#include "RockGazeboHelpers.hpp"
#include <gz/common/Console.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <stdexcept>

#include "Components.hpp"

using namespace std;
using namespace gazebo_usv;

Thrusters::~Thrusters()
{
}

void Thrusters::load(std::shared_ptr<gz::transport::Node> node,
    gz::sim::Entity model,
    sdf::ElementConstPtr plugin_sdf,
    gz::sim::EntityComponentManager& ecm)
{
    m_model = model;
    m_definitions = loadThrusters(model, plugin_sdf, ecm);

    // Initialize communication node and subscribe to gazebo topic
    auto topic_basename = plugin_sdf->Get<string>("topic", "thrusters");
    string topic_name = gz::sim::topicFromScopedName(model, ecm) + "/" + topic_basename.first;

    // Tag the model as having thrusters, and the corresponding topic
    ecm.CreateComponent(m_model, ThrustersTopic(topic_name));
    node->Subscribe(topic_name, &Thrusters::processThrusterCommand, this);

    gzmsg << "Thruster: receiving thruster commands from " << topic_name << endl;
}

Thruster& Thrusters::getThrusterByName(std::string const& name)
{
    for (auto& thruster : m_definitions) {
        if (thruster.getLinkName() == name) {
            return thruster;
        }
    }
    throw std::invalid_argument("no thruster with link " + name);
}

std::vector<Thruster> Thrusters::loadThrusters(gz::sim::Entity model,
    sdf::ElementConstPtr plugin_sdf,
    gz::sim::EntityComponentManager& ecm)
{
    std::vector<Thruster> definitions;
    sdf::ElementConstPtr el = plugin_sdf->FindElement("thruster");
    while (el) {
        // Load thrusters attributes
        Thruster def;
        def.name = el->Get<string>("name");

        auto link = rock_gazebo_helpers::resolveLinkRecursive(model, def.name, ecm);

        gzmsg << "Thruster: thruster " << def.name << " is link "
              << gz::sim::scopedName(link, ecm, "::", false) << endl;
        def.link = link;
        def.min_thrust = rock_gazebo_helpers::getParameter<double>("Thruster",
            el,
            "min_thrust",
            "N",
            -200);
        def.max_thrust = rock_gazebo_helpers::getParameter<double>("Thruster",
            el,
            "max_thrust",
            "N",
            200);
        def.effort = 0.0;
        definitions.push_back(def);
        el = el->GetNextElement("thruster");
    }

    if (definitions.empty()) {
        string msg = "Thruster: sdf model loads thruster plugin but has no\n"
                     "thruster defined. Please name the links you want to export\n"
                     "as thrusters inside the <plugin> tag, e.g.:\n"
                     "<thruster name='thruster::right'> ";
        throw std::invalid_argument(msg);
    }
    return definitions;
}

void Thrusters::processThrusterCommand(gz::gazebo_usv::Thrusters const& thrusters_msg)
{
    for (int i = 0; i < thrusters_msg.thrusters_size(); ++i) {
        bool thruster_found = false;
        const gz::gazebo_usv::Thruster& thruster_cmd = thrusters_msg.thrusters(i);
        for (auto& thruster : m_definitions) {
            if (thruster_cmd.name() == thruster.name) {
                thruster_found = true;
                thruster.effort = thruster_cmd.effort();
                clampThrustEffort(thruster);
            }
        }
        if (!thruster_found) {
            throw std::invalid_argument("Thruster: incoming thruster name: " +
                                        thruster_cmd.name() + ", not found.");
        }
    }
}

void Thrusters::clampThrustEffort(Thruster& thruster)
{
    if (thruster.effort < thruster.min_thrust) {
        gzmsg << "Thruster: thruster effort " << thruster.effort << " below the minimum\n"
              << "Thruster: using min_thrust: " << thruster.min_thrust << " instead"
              << endl;
        thruster.effort = thruster.min_thrust;
    }
    else if (thruster.effort > thruster.max_thrust) {
        gzmsg << "Thruster: thruster effort " << thruster.effort << " above the maximum\n"
              << "Thruster: using max_thrust: " << thruster.max_thrust << " instead"
              << endl;
        thruster.effort = thruster.max_thrust;
    }
}

void Thrusters::update(gz::sim::EntityComponentManager& ecm)
{
    for (auto& thruster : m_definitions) {
        auto thrust = Vector3d::UnitX * thruster.effort;
        gz::sim::Link(thruster.link).AddForceInInertialFrame(ecm, thrust);
    }
}
