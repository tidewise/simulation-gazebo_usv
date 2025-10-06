#include "Utilities.hpp"
#include <gazebo/common/Exception.hh>
#include <regex>

using namespace gazebo_usv;

sdf::ElementPtr utilities::findPluginElement(sdf::ElementPtr enclosing,
    std::string const& file_name)
{
    sdf::ElementPtr plugin_element = enclosing->GetElement("plugin");
    while (plugin_element) {
        if (plugin_element->Get<std::string>("filename") == file_name) {
            gzmsg << "Found plugin: " << plugin_element->Get<std::string>("name") << " ("
                  << file_name << ")" << std::endl;
            return plugin_element;
        }
        plugin_element = plugin_element->GetNextElement("plugin");
    }
    return sdf::ElementPtr();
}

sdf::ElementPtr utilities::findPluginElementByName(sdf::ElementPtr enclosing,
    std::string const& plugin_name)
{
    sdf::ElementPtr plugin_element = enclosing->GetElement("plugin");
    while (plugin_element) {
        if (plugin_element->Get<std::string>("name") == plugin_name) {
            gzmsg << "Found plugin: " << plugin_element->Get<std::string>("name") << " ("
                  << plugin_name << ")" << std::endl;
            return plugin_element;
        }
        plugin_element = plugin_element->GetNextElement("plugin");
    }
    return sdf::ElementPtr();
}

sdf::ElementPtr utilities::getPluginElement(sdf::ElementPtr enclosing,
    std::string const& file_name)
{
    auto element = findPluginElement(enclosing, file_name);

    if (!element) {
        // TODO: change this error message to be more generic
        std::string msg =
            "GazeboThruster: sdf model loaded the thruster plugin, but it\n"
            "cannot be found in the SDF object. Expected the thruster plugin\n"
            "filename to be libgazebo_thruster.so\n";
        gzthrow(msg);
    }

    return element;
}

sdf::ElementPtr utilities::getPluginElementByName(sdf::ElementPtr enclosing,
    std::string const& plugin_name)
{
    auto element = findPluginElementByName(enclosing, plugin_name);

    if (!element) {
        std::string msg =
            "Unable to find any plugin named " + plugin_name + " in the SDF object.\n";
        gzthrow(msg);
    }

    return element;
}

std::string utilities::computeTopicScope(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr plugin)
{
    std::string full_gazebo_name =
        "gazebo::" + model->GetScopedName(true) + "::" + plugin->Get<std::string>("name");
    return "/" + std::regex_replace(full_gazebo_name, std::regex("::"), "/");
}

gazebo::physics::LinkPtr utilities::resolveLink(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr plugin,
    std::string const& link_name)
{
    std::string scope = plugin_name.substr(0, plugin_name.rfind("::"));
    std::string full_link_name = scope + "::" + link_name;
    std::string plugin_name = plugin->Get<std::string>("name");
    auto link = model->GetLink(full_link_name);
    if (!link) {
        std::string msg =
            "could not find link " + link_name + " specified in plugin " +
            plugin_name + " (full link name resolved to " + full_link_name + ")";
        gzthrow(msg);
    }
    return link;
}

gazebo::physics::LinkPtr utilities::resolveLinkWithDefault(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr plugin_sdf,
    std::string const& element_name
) {
    if (plugin_sdf->HasElement(element_name))
    {
        auto link_name = plugin_sdf->Get<std::string>(element_name);
        return utilities::resolveLink(model, plugin_sdf, link_name);
    }
    else if (model->GetLinks().empty())
    {
        gzthrow("No link defined in reference model, and no " + element_name + " element found plugin");
    }
    else
    {
        auto link = model->GetLinks().front();
        gzmsg << "Element " << element_name << " missing in plugin definition, using "
              << link->GetScopedName() << " instead!" << std::endl;
        return link;
    }
}
