#include "Utilities.hpp"
#include <gazebo/common/Exception.hh>

using namespace gazebo_usv;

sdf::ElementPtr utilities::findPluginElement(
    sdf::ElementPtr enclosing, std::string const& fileName
) {
    sdf::ElementPtr pluginElement = enclosing->GetElement("plugin");
    while (pluginElement)
    {
        if (pluginElement->Get<std::string>("filename") == fileName) {
            gzmsg << "Found plugin: " << pluginElement->Get<std::string>("name")
                  << " (" << fileName << ")" << std::endl;
            return pluginElement;
        }
        pluginElement = pluginElement->GetNextElement("plugin");
    }
    return sdf::ElementPtr();
}

sdf::ElementPtr utilities::findPluginElementByName(
    sdf::ElementPtr enclosing, std::string const& pluginName
) {
    sdf::ElementPtr pluginElement = enclosing->GetElement("plugin");
    while (pluginElement)
    {
        if (pluginElement->Get<std::string>("name") == pluginName) {
            gzmsg << "Found plugin: " << pluginElement->Get<std::string>("name")
                  << " (" << pluginName << ")" << std::endl;
            return pluginElement;
        }
        pluginElement = pluginElement->GetNextElement("plugin");
    }
    return sdf::ElementPtr();
}

sdf::ElementPtr utilities::getPluginElement(
    sdf::ElementPtr enclosing, std::string const& fileName
) {
    auto element = findPluginElement(enclosing, fileName);

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

sdf::ElementPtr utilities::getPluginElementByName(
    sdf::ElementPtr enclosing, std::string const& pluginName
) {
    auto element = findPluginElementByName(enclosing, pluginName);

    if (!element) {
        std::string msg =
            "Unable to find any plugin named " + pluginName + " in the SDF object.\n";
        gzthrow(msg);
    }

    return element;
}