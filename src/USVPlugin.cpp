#include <gazebo_usv/USVPlugin.hpp>
#include "Utilities.hpp"

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;

USVPlugin::~USVPlugin() {
    delete mActuators;
}

void USVPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    mModel = _model;
    // Import all thrusters from a model file (sdf)
    sdf::ElementPtr modelSDF = mModel->GetSDF();
    sdf::ElementPtr pluginElement = utilities::getPluginElement(
        modelSDF, "libgazebo_usv.so"
    );

    // Initialize communication node and subscribe to gazebo topic
    mNode = transport::NodePtr(new transport::Node());
    mNode->Init();

    mActuators = new Actuators(mModel, mNode);
    mThrusters = new Thrusters;

    mWorldUpdateEvent = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&USVPlugin::updateBegin, this, _1)
    );

    loadRudders(pluginElement);
    loadThrusters(pluginElement);
}

Rudder& USVPlugin::getRudderByName(std::string const& name) {
    auto rudder_it = find_if(
        mRudders.begin(), mRudders.end(),
        [name](Rudder& r) { return r.getLinkName() == name; }
    );
    if (rudder_it == mRudders.end()) {
        throw std::invalid_argument("no rudder named " + name);
    }
    return *rudder_it;

}

void USVPlugin::loadRudders(sdf::ElementPtr pluginElement) {
    sdf::ElementPtr el = pluginElement->GetElement("rudder");
    while (el) {
        mRudders.push_back(Rudder(*mActuators, mModel, el));
        el = el->GetNextElement("rudder");
    }
}

void USVPlugin::loadThrusters(sdf::ElementPtr pluginElement) {
    sdf::ElementPtr el = pluginElement->GetElement("thruster");
    if (el) {
        mThrusters->load(*this, *mActuators, mNode, mModel, pluginElement);
    }
}

void USVPlugin::updateBegin(common::UpdateInfo const& info) {
    for (auto& rudder : mRudders) {
        rudder.update(*mActuators);
    }

    mThrusters->update(*mActuators);
}

GZ_REGISTER_MODEL_PLUGIN(USVPlugin);