#include <gazebo_usv/USVPlugin.hpp>

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;

USVPlugin::~USVPlugin() {
    delete m_wind;
    delete m_wave;
    delete m_thrusters;
    delete m_actuators;
    delete m_direct_force;
}

void USVPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _plugin_sdf)
{
    m_model = _model;

    m_node = transport::NodePtr(new transport::Node());
    m_node->Init();

    m_actuators = new Actuators(m_model, m_node);

    m_world_update_event = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&USVPlugin::updateBegin, this, _1)
    );

    auto plugin_name = _plugin_sdf->Get<std::string>("name");
    if (plugin_name.find("thrusters") != std::string::npos) {
        m_thrusters = loadThrusters(_plugin_sdf);
        m_rudders = loadRudders(_plugin_sdf);
    }
    else if (plugin_name.find("wind_dynamics") != std::string::npos) {
        m_wind = loadWindParameters(_plugin_sdf);
    }
    else if (plugin_name.find("wave_dynamics") != std::string::npos) {
        m_wave = loadWaveParameters(_plugin_sdf);
    }
    else if (plugin_name.find("direct_force") != std::string::npos) {
        m_direct_force = loadDirectForceApplicationParameters(_plugin_sdf);
    }
}

Rudder& USVPlugin::getRudderByName(std::string const& name) {
    for (auto& rudder : m_rudders) {
        if (rudder.getLinkName() == name) {
            return rudder;
        }
    }
    gzthrow("no rudder named " + name);
}

Thruster& USVPlugin::getThrusterByName(std::string const& name) {
    return m_thrusters->getThrusterByName(name);
}

std::vector<Rudder> USVPlugin::loadRudders(sdf::ElementPtr thrusters_plugin_element) {
    if (!thrusters_plugin_element || !thrusters_plugin_element->HasElement("rudder")) {
        return {};
    }

    std::vector<Rudder> rudders;

    sdf::ElementPtr el = thrusters_plugin_element->GetElement("rudder");
    std::string plugin_name = thrusters_plugin_element->Get<string>("name");
    while (el) {
        rudders.push_back(Rudder(*this, *m_actuators, m_model, el, plugin_name));
        el = el->GetNextElement("rudder");
    }

    return rudders;
}

Thrusters* USVPlugin::loadThrusters(sdf::ElementPtr thrusters_plugin_element) {
    if (!thrusters_plugin_element || !thrusters_plugin_element->HasElement("thruster")) {
        return nullptr;
    }

    Thrusters* thrusters = new Thrusters;
    thrusters->load(*m_actuators, m_node, m_model, thrusters_plugin_element);
    return thrusters;
}

Wind* USVPlugin::loadWindParameters(sdf::ElementPtr wind_plugin_element) {
    if (!wind_plugin_element) {
        return nullptr;
    }

    Wind* wind = new Wind;
    wind->load(m_model, m_node, wind_plugin_element);

    return wind;
}

Wave* USVPlugin::loadWaveParameters(sdf::ElementPtr wave_plugin_element) {
    if (!wave_plugin_element) {
        return nullptr;
    }

    Wave* wave = new Wave;
    wave->load(m_model, m_node, wave_plugin_element);

    return wave;
}

DirectForceApplication* USVPlugin::loadDirectForceApplicationParameters(
    sdf::ElementPtr direct_force_plugin_element)
{
    if (!direct_force_plugin_element) {
        return nullptr;
    }

    DirectForceApplication* direct_force = new DirectForceApplication;
    direct_force->load(*m_actuators, m_model, m_node, direct_force_plugin_element);

    return direct_force;
}


void USVPlugin::updateBegin(common::UpdateInfo const& info) {
    for (auto& rudder : m_rudders) {
        rudder.update(*m_actuators);
    }

    if (m_thrusters) {
        m_thrusters->update(*m_actuators);
    }

    if (m_wind) {
        m_wind->update();
    }

    if (m_wave) {
        m_wave->update();
    }

    if (m_direct_force) {
        m_direct_force->update(*m_actuators);
    }
}

GZ_REGISTER_MODEL_PLUGIN(USVPlugin);
