#include <gazebo_usv/DirectForceApplication.hpp>

#include <string>

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <gazebo_usv/Actuators.hpp>

using namespace gazebo_usv;

DirectForceApplication::~DirectForceApplication() {
    if (mCommandSubscriber) {
        mCommandSubscriber->Unsubscribe();
    }
}

void DirectForceApplication::load(
    Actuators& actuators, gazebo::physics::ModelPtr model,
    gazebo::transport::NodePtr node, sdf::ElementPtr pluginElement
) {
    auto link_name = pluginElement->Get<std::string>("link");

    if (link_name.empty()) {
        std::string msg = "DirectForceApplication: sdf model loads 'gazebo_usv_force' plugin,\n"
                      "but does not defines a link parameter. Please name the link\n"
                      "you want to apply a force inside the <plugin> tag, e.g.:\n"
                      "<link>'link_1'</link> ";
        gzthrow(msg);
    }

    auto link = model->GetLink(link_name);
    if (!link) {
        std::string msg = "DirectForceApplication: sdf model loads 'direct_force' plugin,\n"
                          "but it's defining an invalid link name as parameter. Please make sure\n"
                          "that the link you're naming actually exists in the model's sdf.";
        gzthrow(msg);
    }

    mLinkId = actuators.addLink(link);

    // Initialize communication node and subscribe to gazebo topic
    std::string topicName = model->GetName() + "/" + link->GetName() + "/gazebo_usv_force";
    if (mCommandSubscriber) {
        mCommandSubscriber->Unsubscribe();
    }
    mCommandSubscriber =
        node->Subscribe("~/" + topicName, &DirectForceApplication::processDirectionalForceCommand, this);

    auto worldName = model->GetWorld()->Name();
    gzmsg << "DirectForceApplication: receiving directioned force commands from /gazebo/"
          << worldName << "/" << model->GetName() << "/" << link->GetName() << "/gazebo_usv_force" << std::endl;
}

void DirectForceApplication::processDirectionalForceCommand(ConstVector3dPtr const& force_msg) {
    mForceCmd.X() = force_msg->has_x() ? force_msg->x() : 0.;
    mForceCmd.Y() = force_msg->has_y() ? force_msg->y() : 0.;
    mForceCmd.Z() = force_msg->has_z() ? force_msg->z() : 0.;
}

void DirectForceApplication::update(Actuators& actuators) {
    actuators.applyForce(mLinkId, mForceCmd);
}

