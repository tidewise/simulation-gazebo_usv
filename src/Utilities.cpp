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

std::string utilities::computeModelTopicScope(
    gazebo::physics::ModelPtr model)
{
    return "/gazebo/" + std::regex_replace(model->GetScopedName(true), std::regex("::"), "/");
}

std::string utilities::computePluginTopicScope(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr plugin)
{
    std::string plugin_name = plugin->Get<std::string>("name");
    if (plugin_name.find("__") != std::string::npos) {
        return "/" + std::regex_replace(plugin_name, std::regex("__"), "/");
    }
    else {
        std::string full_gazebo_name =
            "gazebo::" + model->GetScopedName(true) + "::" + plugin_name;
        return "/" + std::regex_replace(full_gazebo_name, std::regex("::"), "/");
    }
}

static std::string resolveLinkScopeFromDoubleUndescoredPluginName(
    gazebo::physics::ModelPtr model,
    std::string const& plugin_name
) {
    // Do slightly better than the actual old implementation, and validate that
    // the beginning of the full path is the fully scoped model name
    auto expected_prefix_gazebo = "gazebo::" + model->GetScopedName(true);
    auto expected_prefix = std::regex_replace(expected_prefix_gazebo, std::regex("::"), "__");
    if (plugin_name.substr(0, expected_prefix.size()) != expected_prefix) {
        gzthrow("expected " + plugin_name + " to start with " + expected_prefix);
    }

    auto rfind = plugin_name.rfind("__");
    if (rfind == expected_prefix.size()) {
        return "";
    }
    auto relative_scope = plugin_name.substr(
        expected_prefix.size() + 2, rfind - expected_prefix.size() - 2
    );

    return std::regex_replace(relative_scope, std::regex("__"), "::");
}

static std::string resolvePluginParentScope(
    gazebo::physics::ModelPtr model, std::string const& plugin_name
) {
    if (plugin_name.find("__") != std::string::npos) {
        return resolveLinkScopeFromDoubleUndescoredPluginName(model, plugin_name);
    }
    else {
        return plugin_name.substr(0, plugin_name.rfind("::"));
    }
}

static std::string applyScope(std::string const& scope, std::string const& name) {
    if (scope.empty()) {
        return name;
    } else {
        return scope + "::" + name;
    }
}

gazebo::physics::LinkPtr utilities::resolveLink(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr plugin,
    std::string const& link_name)
{
    std::string plugin_name = plugin->Get<std::string>("name");

    auto scope = resolvePluginParentScope(model, plugin_name);
    auto full_link_name = applyScope(scope, link_name);

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
        return resolveLink(model, plugin_sdf, link_name);
    }
    else if (model->GetLinks().empty())
    {
        gzthrow("No link defined in reference model, and no " + element_name + " element found plugin");
    }
    else
    {
        auto plugin_name = plugin_sdf->Get<std::string>("name");
        auto scope = resolvePluginParentScope(model, plugin_name);
        auto it = std::find_if(
            model->GetLinks().begin(),
            model->GetLinks().end(),
            [&](auto link_ptr) -> bool {
                auto s = link_ptr->GetName();
                return s.substr(0, scope.size()) == scope;
            }
        );

        if (it == model->GetLinks().end()) {
            gzthrow(
                "Element " + element_name + " missing in plugin definition, attempted "
                "to find a link whose scoped name starts with " + scope + " but found "
                "none"
            );
        }

        gzmsg <<
            "Element " << element_name << " missing in plugin definition, picking first "
            "link of the enclosing model, " << (*it)->GetScopedName() << std::endl;
        return *it;
    }
}
