#include "RockGazeboHelpers.hpp"
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <stdexcept>

using namespace std;

string rock_gazebo_helpers::computeModelTopicScope(gz::sim::Entity model, gz::sim::EntityComponentManager& ecm)
{
    return "/gazebo/" + gz::sim::scopedName(model, ecm, "/", false);
}

string rock_gazebo_helpers::computePluginTopicScope(gz::sim::Entity model,
    sdf::ElementConstPtr plugin, gz::sim::EntityComponentManager& ecm)
{
    auto topic_name = plugin->Get<string>("thrusters_topic_name", "thrusters");
    return computeModelTopicScope(model, plugin, ecm) + "/" + topic_name.first;
}

gz::sim::Entity rock_gazebo_helpers::resolveLinkWithDefault(
    gz::sim::Entity model,
    sdf::ElementConstPtr plugin_sdf,
    string const& element_name,
    gz::sim::EntityComponentManager& ecm
)
{
    if (plugin_sdf->HasElement(element_name)) {
        auto link_name = plugin_sdf->Get<string>(element_name);
        return resolveLinkRecursive(model, link_name, ecm);
    }

    auto links = gz::sim::Model(model).Links(ecm);
    if (links.empty()) {
        throw std::invalid_argument("No link defined in reference model, and no " + element_name +
                " element found plugin");
    }

    return links.front();
}
