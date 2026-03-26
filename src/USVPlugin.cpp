#include <gazebo_usv/USVPlugin.hpp>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport.hh>

using namespace std;
using namespace gazebo_usv;
using namespace gz;
using namespace gz::sim;

USVPlugin::~USVPlugin()
{
    delete m_wind;
    delete m_wave;
    delete m_thrusters;
    delete m_direct_force;
}

void USVPlugin::Configure(gz::sim::Entity const& entity,
    std::shared_ptr<const sdf::Element> const& plugin_sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_manager)
{
    gzmsg << "Loading USVPlugin" << std::endl;
    m_model = entity;

    m_node.reset(new transport::Node());

    auto thrusters_sdf = plugin_sdf->FindElement("thrusters");
    if (thrusters_sdf) {
        m_thrusters = loadThrusters(thrusters_sdf, ecm);
    }

    auto rudders_sdf = plugin_sdf->FindElement("rudders");
    if (rudders_sdf) {
        m_rudders = loadRudders(rudders_sdf, ecm);
    }

    auto wind_sdf = plugin_sdf->FindElement("wind_dynamics");
    if (wind_sdf) {
        m_wind = loadWindParameters(wind_sdf, ecm);
    }

    auto wave_sdf = plugin_sdf->FindElement("wave_dynamics");
    if (wave_sdf) {
        m_wave = loadWaveParameters(wave_sdf, ecm);
    }

    auto direct_force_sdf = plugin_sdf->FindElement("direct_force");
    if (direct_force_sdf) {
        m_direct_force = loadDirectForceApplicationParameters(direct_force_sdf, ecm);
    }
}

Rudder& USVPlugin::getRudderByName(std::string const& name)
{
    for (auto& rudder : m_rudders) {
        if (rudder.getLinkName() == name) {
            return rudder;
        }
    }
    throw std::invalid_argument("no rudder named " + name);
}

Thruster& USVPlugin::getThrusterByName(std::string const& name)
{
    return m_thrusters->getThrusterByName(name);
}

std::vector<Rudder> USVPlugin::loadRudders(sdf::ElementConstPtr plugin_sdf,
    gz::sim::EntityComponentManager& ecm)
{
    std::vector<Rudder> rudders;

    sdf::ElementConstPtr el = plugin_sdf->FindElement("rudder");
    while (el) {
        rudders.push_back(Rudder(*this, m_model, el, ecm));
        el = el->GetNextElement("rudder");
    }

    return rudders;
}

Thrusters* USVPlugin::loadThrusters(sdf::ElementConstPtr plugin_sdf,
    gz::sim::EntityComponentManager& ecm)
{
    if (!plugin_sdf || !plugin_sdf->HasElement("thruster")) {
        return nullptr;
    }

    Thrusters* thrusters = new Thrusters;
    thrusters->load(m_node, m_model, plugin_sdf, ecm);
    return thrusters;
}

Wind* USVPlugin::loadWindParameters(sdf::ElementConstPtr plugin_sdf,
    gz::sim::EntityComponentManager& ecm)
{
    if (!plugin_sdf) {
        return nullptr;
    }

    Wind* wind = new Wind;
    wind->load(m_model, m_node, plugin_sdf, ecm);
    return wind;
}

Wave* USVPlugin::loadWaveParameters(sdf::ElementConstPtr plugin_sdf,
    gz::sim::EntityComponentManager& ecm)
{
    if (!plugin_sdf) {
        return nullptr;
    }

    Wave* wave = new Wave;
    wave->load(m_model, m_node, plugin_sdf, ecm);
    return wave;
}

DirectForceApplication* USVPlugin::loadDirectForceApplicationParameters(
    sdf::ElementConstPtr plugin_sdf,
    gz::sim::EntityComponentManager& ecm)
{
    if (!plugin_sdf) {
        return nullptr;
    }

    DirectForceApplication* direct_force = new DirectForceApplication;
    direct_force->load(m_model, m_node, plugin_sdf, ecm);

    return direct_force;
}

void USVPlugin::PreUpdate(gz::sim::UpdateInfo const& info,
    gz::sim::EntityComponentManager& ecm)
{
    for (auto& rudder : m_rudders) {
        rudder.update(ecm);
    }

    if (m_thrusters) {
        m_thrusters->update(ecm);
    }

    if (m_wind) {
        m_wind->update(ecm);
    }

    if (m_wave) {
        m_wave->update(ecm);
    }

    if (m_direct_force) {
        m_direct_force->update(ecm);
    }
}

GZ_ADD_PLUGIN(
    USVPlugin,
    gz::sim::System,
    USVPlugin::ISystemConfigure,
    USVPlugin::ISystemPreUpdate
);
