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

std::string utilities::getNamespaceFromPluginName(std::string const& plugin_name)
{
    return std::regex_replace(plugin_name, std::regex("__"), "/");
}
gazebo::physics::LinkPtr utilities::getLinkFromName(gazebo::physics::ModelPtr model,
    std::string const& link_name,
    std::string const& plugin_name)
{
    std::string link_name_iterated = link_name;
    std::string plugin_name_iterated = plugin_name.substr(0, plugin_name.rfind("__"));
    std::size_t pos = plugin_name_iterated.rfind("__");
    std::string plugin_prefix = plugin_name_iterated.substr(pos + 2);
    auto link = model->GetLink(link_name_iterated);

    while (pos != std::string::npos) {
        if (link) {
            break;
        }
        pos = plugin_name_iterated.rfind("__");
        plugin_prefix = plugin_name_iterated.substr(pos + 2);
        plugin_name_iterated = plugin_name_iterated.substr(0, pos);
        link_name_iterated = plugin_prefix + "::" + link_name_iterated;
        link = model->GetLink(link_name_iterated);
    }

    return link;
}
