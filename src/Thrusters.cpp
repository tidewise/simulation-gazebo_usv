#include "Thrusters.hpp"
#include "Utilities.hpp"
#include "Actuators.hpp"
#include "USVPlugin.hpp"

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;
using namespace ignition::math;

Thrusters::~Thrusters() {
    if (mCommandSubscriber) {
        mCommandSubscriber->Unsubscribe();
    }
}

void Thrusters::load(
    Actuators& actuators, transport::NodePtr node,
    physics::ModelPtr model, sdf::ElementPtr pluginElement
) {
    mModel = model;
    mDefinitions = loadThrusters(actuators, pluginElement);

    // Initialize communication node and subscribe to gazebo topic
    auto pluginName = pluginElement->Get<std::string>("name");
    string topicName = utilities::getTopicNameFromPluginName(pluginName) + "/thrusters";

    if (mCommandSubscriber) {
        mCommandSubscriber->Unsubscribe();
    }
    mCommandSubscriber =
        node->Subscribe("/" + topicName, &Thrusters::processThrusterCommand, this);

    auto worldName = GzGet((*mModel->GetWorld()), Name, ());
    gzmsg << "Thruster: receiving thruster commands from /"
          << topicName << endl;
}

Thruster& Thrusters::getThrusterByName(std::string const& name) {
    for (auto& thruster : mDefinitions) {
        if (thruster.getLinkName() == name) {
            return thruster;
        }
    }
    gzthrow("no thruster with link " + name);
}

std::vector<Thruster> Thrusters::loadThrusters(
    Actuators& actuators, sdf::ElementPtr pluginElement
) {
    std::vector<Thruster> definitions;
    sdf::ElementPtr el = pluginElement->GetElement("thruster");
    while (el) {
        // Load thrusters attributes
        Thruster def;
        def.name = el->Get<string>("name");
        auto link = mModel->GetLink(def.name);
        if (!link) {
            gzthrow("Thruster: thruster " + def.name + " does not exist");
        }

        gzmsg << "Thruster: thruster name: " << def.name << endl;
        def.actuatorID = actuators.addLink(link);
        def.link = link;
        def.minThrust = utilities::getParameter<double>(
            "Thruster", el, "min_thrust", "N", -200
        );
        def.maxThrust = utilities::getParameter<double>(
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

void Thrusters::processThrusterCommand(ThrustersMSG const& thrustersMSG) {
    for (int i = 0; i < thrustersMSG->thrusters_size(); ++i) {
        bool thrusterFound = false;
        const gazebo_thruster::msgs::Thruster& thrusterCMD = thrustersMSG->thrusters(i);
        for (auto& thruster : mDefinitions) {
            if (thrusterCMD.name() == thruster.name) {
                thrusterFound = true;
                thruster.effort = thrusterCMD.has_effort() ? thrusterCMD.effort() : 0;
                clampThrustEffort(thruster);
            }
        }
        if (!thrusterFound) {
            gzthrow("Thruster: incoming thruster name: "
                    + thrusterCMD.name() + ", not found.");
        }
    }
}

void Thrusters::clampThrustEffort(Thruster& thruster)
{
    if (thruster.effort < thruster.minThrust) {
        gzmsg << "Thruster: thruster effort " << thruster.effort
              << " below the minimum\n"
              << "Thruster: using minThrust: " << thruster.minThrust
              << " instead" << endl;
        thruster.effort = thruster.minThrust;
    }
    else if (thruster.effort > thruster.maxThrust)
    {
        gzmsg << "Thruster: thruster effort " << thruster.effort
              << " above the maximum\n"
              << "Thruster: using maxThrust: " << thruster.maxThrust
              << " instead" << endl;
        thruster.effort = thruster.maxThrust;
    }
}

void Thrusters::update(Actuators& actuators) {
    for (auto& thruster : mDefinitions) {
        auto thrust = Vector3d::UnitX * thruster.effort;
        actuators.applyForce(thruster.actuatorID, thrust);
    }
}
