#include <gazebo_usv/USVPlugin.hpp>

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;

USVPlugin::~USVPlugin() {
    delete mWind;
    delete mThrusters;
    delete mActuators;
    delete mDirectForce;
}

void USVPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _plugin_sdf)
{
    mModel = _model;

    mNode = transport::NodePtr(new transport::Node());
    mNode->Init();

    mActuators = new Actuators(mModel, mNode);

    mWorldUpdateEvent = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&USVPlugin::updateBegin, this, _1)
    );

    auto pluginName = _plugin_sdf->Get<std::string>("name");
    if ("thrusters" == pluginName) {
        mThrusters = loadThrusters(_plugin_sdf);
        mRudders = loadRudders(_plugin_sdf);
    }
    else if ("wind_dynamics" == pluginName) {
        mWind = loadWindParameters(_plugin_sdf);
    }
    else if ("direct_force" == pluginName) {
        mDirectForce = loadDirectForceApplicationParameters(_plugin_sdf);
    }
}

Rudder& USVPlugin::getRudderByName(std::string const& name) {
    for (auto& rudder : mRudders) {
        if (rudder.getLinkName() == name) {
            return rudder;
        }
    }
    gzthrow("no rudder named " + name);
}

Thruster& USVPlugin::getThrusterByName(std::string const& name) {
    return mThrusters->getThrusterByName(name);
}

std::vector<Rudder> USVPlugin::loadRudders(sdf::ElementPtr thrustersPluginElement) {
    if (!thrustersPluginElement || !thrustersPluginElement->HasElement("rudder")) {
        return {};
    }

    std::vector<Rudder> rudders;

    sdf::ElementPtr el = thrustersPluginElement->GetElement("rudder");
    while (el) {
        rudders.push_back(Rudder(*this, *mActuators, mModel, el));
        el = el->GetNextElement("rudder");
    }

    return rudders;
}

Thrusters* USVPlugin::loadThrusters(sdf::ElementPtr thrustersPluginElement) {
    if (!thrustersPluginElement || !thrustersPluginElement->HasElement("thruster")) {
        return nullptr;
    }

    Thrusters* thrusters = new Thrusters;
    thrusters->load(*mActuators, mNode, mModel, thrustersPluginElement);
    return thrusters;
}

Wind* USVPlugin::loadWindParameters(sdf::ElementPtr windPluginElement) {
    if (!windPluginElement) {
        return nullptr;
    }

    Wind* wind = new Wind;
    wind->load(mModel, mNode, windPluginElement);

    return wind;
}

DirectForceApplication* USVPlugin::loadDirectForceApplicationParameters(
    sdf::ElementPtr directForcePluginElement)
{
    if (!directForcePluginElement) {
        return nullptr;
    }

    DirectForceApplication* direct_force = new DirectForceApplication;
    direct_force->load(*mActuators, mModel, mNode, directForcePluginElement);

    return direct_force;
}


void USVPlugin::updateBegin(common::UpdateInfo const& info) {
    for (auto& rudder : mRudders) {
        rudder.update(*mActuators);
    }

    if (mThrusters) {
        mThrusters->update(*mActuators);
    }

    if (mWind) {
        mWind->update();
    }

    if (mDirectForce) {
        mDirectForce->update(*mActuators);
    }
}

GZ_REGISTER_MODEL_PLUGIN(USVPlugin);
