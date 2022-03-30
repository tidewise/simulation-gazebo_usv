#include <gazebo_usv/USVPlugin.hpp>
#include "Utilities.hpp"

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;

USVPlugin::~USVPlugin() {
    delete mWind;
    delete mThrusters;
    delete mActuators;
}

void USVPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    mModel = _model;
    // Import all thrusters from a model file (sdf)
    sdf::ElementPtr modelSDF = mModel->GetSDF();
    sdf::ElementPtr thrustersPluginElement = utilities::getPluginElementByName(
        modelSDF, "thrusters"
    );
    // Since the wind plugin is optional and uses the same filename as the thrusters plugin, it should be found by name and be ignored if not found 
    sdf::ElementPtr windPluginElement = utilities::findPluginElementByName(modelSDF, "wind_dynamics");

    // Initialize communication node and subscribe to gazebo topic
    mNode = transport::NodePtr(new transport::Node());
    mNode->Init();

    mActuators = new Actuators(mModel, mNode);
    mThrusters = new Thrusters;

    mWorldUpdateEvent = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&USVPlugin::updateBegin, this, _1)
    );

    loadThrusters(thrustersPluginElement);
    loadRudders(thrustersPluginElement);

    if (windPluginElement) {
        mWind = new Wind;
        loadWindParameters(windPluginElement);
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

void USVPlugin::loadRudders(sdf::ElementPtr pluginElement) {
    sdf::ElementPtr el = pluginElement->GetElement("rudder");
    while (el) {
        mRudders.push_back(Rudder(*this, *mActuators, mModel, el));
        el = el->GetNextElement("rudder");
    }
}

void USVPlugin::loadThrusters(sdf::ElementPtr pluginElement) {
    sdf::ElementPtr el = pluginElement->GetElement("thruster");
    if (el) {
        mThrusters->load(*mActuators, mNode, mModel, pluginElement);
    }
}

void USVPlugin::loadWindParameters(sdf::ElementPtr pluginElement){
       mWind->load(mModel, mNode, pluginElement);
}

void USVPlugin::updateBegin(common::UpdateInfo const& info) {
    for (auto& rudder : mRudders) {
        rudder.update(*mActuators);
    }

    mThrusters->update(*mActuators);

    if (mWind) {
        mWind->update();
    }
}

GZ_REGISTER_MODEL_PLUGIN(USVPlugin);