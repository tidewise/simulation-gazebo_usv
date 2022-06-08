#include <gazebo_usv/USVPlugin.hpp>
#include "Utilities.hpp"

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;

USVPlugin::~USVPlugin() {
    delete mWind;
    delete mThrusters;
    delete mActuators;
    delete mDirectForce;
}

void USVPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    mModel = _model;
    // Import all thrusters from a model file (sdf)
    sdf::ElementPtr modelSDF = mModel->GetSDF();
    sdf::ElementPtr thrustersPluginElement = utilities::getPluginElementByName(
        modelSDF, "thrusters"
    );

    // Since the following plugins are optional and uses the same filename as the thrusters plugin,
    // it should be found by name and be ignored if not found
    sdf::ElementPtr windPluginElement = utilities::findPluginElementByName(modelSDF, "wind_dynamics");
    sdf::ElementPtr directForcePluginElement = utilities::findPluginElementByName(modelSDF, "direct_force");

    // Initialize communication node and subscribe to gazebo topic
    mNode = transport::NodePtr(new transport::Node());
    mNode->Init();

    mActuators = new Actuators(mModel, mNode);

    mWorldUpdateEvent = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&USVPlugin::updateBegin, this, _1)
    );

    mThrusters = loadThrusters(thrustersPluginElement);
    mRudders = loadRudders(thrustersPluginElement);
    mWind = loadWindParameters(windPluginElement);
    mDirectForce = loadDirectForceApplicationParameters(directForcePluginElement);
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
    if (!thrustersPluginElement->HasElement("rudder")) {
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
    Thrusters* thrusters = new Thrusters;

    if (sdf::ElementPtr el = thrustersPluginElement->GetElement("thruster")) {
        thrusters->load(*mActuators, mNode, mModel, thrustersPluginElement);
    }

    return thrusters;
}

Wind* USVPlugin::loadWindParameters(sdf::ElementPtr windPluginElement){
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

    mThrusters->update(*mActuators);

    if (mWind) {
        mWind->update();
    }

    if (mDirectForce) {
        mDirectForce->update(*mActuators);
    }
}

GZ_REGISTER_MODEL_PLUGIN(USVPlugin);
