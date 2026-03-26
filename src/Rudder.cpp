#include <gazebo_usv/Rudder.hpp>
#include <gazebo_usv/Thruster.hpp>
#include <gazebo_usv/USVPlugin.hpp>
#include <gazebo_usv/RockGazeboHelpers.hpp>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Link.hh>

using namespace std;
using namespace gz::sim;
using namespace gazebo_usv;
using namespace gz::math;

Rudder::Rudder(USVPlugin& plugin, gz::sim::Entity model, sdf::ElementConstPtr rudder_sdf, gz::sim::EntityComponentManager& ecm) {
    auto plugin_sdf = rudder_sdf->GetParent();
    m_link_name = rudder_sdf->Get<string>("name");
    m_link = rock_gazebo_helpers::resolveLinkRecursive(model, m_link_name, ecm);
    gzmsg << "Rudder: resolved rudder link " << gz::sim::scopedName(m_link, ecm, "::", false) << std::endl;

    auto thruster_name = rudder_sdf->Get<string>("thrusterName");
    if (!thruster_name.empty()) {
        m_associated_thruster = &plugin.getThrusterByName(thruster_name);
        m_thrust_to_speed_k = rudder_sdf->Get<float>("thrustToFlowK", 0).first;
    }

    m_area = rudder_sdf->Get<float>("area", 1).first;
    m_lift_k = rudder_sdf->Get<float>("lift_factor", 1.5).first;
    m_drag_k = rudder_sdf->Get<float>("drag_factor", 1e-3).first;
}

Rudder::~Rudder() {
}

std::string Rudder::getLinkName() const {
    return m_link_name;
}

Vector3d Rudder::getFlowVelocity(gz::sim::EntityComponentManager& ecm) const {
    if (!m_associated_thruster) {
        return Link(m_link).WorldLinearVelocity(ecm).value();
    }

    auto thruster_pose = m_associated_thruster->getWorldPose(ecm);
    auto thruster_x = thruster_pose.Rot().RotateVector(Vector3d::UnitX);
    auto advance_speed = m_associated_thruster->getAdvanceSpeed(ecm);

    float velocity_x_sq = advance_speed * abs(advance_speed) +
                        m_thrust_to_speed_k * m_associated_thruster->getEffort();
    return copysign(sqrt(abs(velocity_x_sq)), velocity_x_sq) * thruster_x;
}

void Rudder::update(gz::sim::EntityComponentManager& ecm) {
    // get linear velocity at cp in inertial frame
    auto vel = getFlowVelocity(ecm);
    if (vel.Length () <= 0.01) {
        return;
    }

    // pose of body
    auto pose = Link(m_link).WorldPose(ecm).value();

    // rotate forward and upward vectors into inertial frame
    auto forward_i = pose.Rot().RotateVector(Vector3d::UnitX);
    auto upward_i = pose.Rot().RotateVector(Vector3d::UnitZ);

    // ld_normal vector to lift-drag-plane described in inertial frame
    auto ld_normal = forward_i.Cross(upward_i).Normalize();

    // angle of attack
    auto vel_in_ld_plane = ld_normal.Cross(vel.Cross(ld_normal));

    // get direction of drag
    auto drag_direction = -vel_in_ld_plane.Normalized();

    // get direction of lift
    auto liftDirection = ld_normal.Cross(vel_in_ld_plane).Normalized();

    float alpha = atan2(-upward_i.Dot(vel_in_ld_plane), forward_i.Dot(vel_in_ld_plane));

    // compute dynamic pressure
    double speed_in_ld_plane = vel_in_ld_plane.Length();
    double q = 0.5 * m_fluid_density * speed_in_ld_plane * speed_in_ld_plane;

    // compute cl at cp, check for stall, correct for sweep
    double cl = m_lift_k * sin(2*alpha);
    // compute lift force at cp
    auto lift = cl * q * m_area * liftDirection;

    // compute cd at cp, check for stall, correct for sweep
    double cd = fabs(m_drag_k * (1 - cos(2 * alpha)));

    // drag at cp
    auto drag = cd * q * m_area * drag_direction;

    Vector3d world_force = lift + drag;
    Link(m_link).AddWorldForce(ecm, world_force);
}
