#include <gazebo_usv/Actuators.hpp>
#include <gazebo_usv/Rudder.hpp>
#include <gazebo_usv/Thruster.hpp>
#include <gazebo_usv/USVPlugin.hpp>
#include <gazebo_usv/Utilities.hpp>

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;
using namespace ignition::math;

Rudder::Rudder(USVPlugin& plugin, Actuators& actuators, physics::ModelPtr model, sdf::ElementPtr sdf) {
    m_link_name = sdf->Get<string>("name");
    m_link = model->GetLink(m_link_name);
    if (!m_link) {
        gzthrow("Rudder: link " + m_link_name + " does not exist");
    }

    auto thruster_name = sdf->Get<string>("thrusterName");
    if (!thruster_name.empty()) {
        m_associated_thruster = &plugin.getThrusterByName(thruster_name);
        m_thrust_to_speed_k = sdf->Get<float>("thrustToFlowK", 0).first;
    }

    m_actuator_id = actuators.addLink(m_link);

    m_area = sdf->Get<float>("area", 1).first;
    m_lift_k = sdf->Get<float>("lift_factor", 1.5).first;
    m_drag_k = sdf->Get<float>("drag_factor", 1e-3).first;
}

Rudder::~Rudder() {
}

std::string Rudder::getLinkName() const {
    return m_link_name;
}

Vector3d Rudder::getFlowVelocity() const {
    if (!m_associated_thruster) {
        return m_link->WorldLinearVel();
    }

    auto thruster_velocity = m_associated_thruster->getLink()->WorldLinearVel();
    auto thruster_pose = m_associated_thruster->getLink()->WorldPose();
    auto thruster_x = thruster_pose.Rot().RotateVector(Vector3d::UnitX);
    auto advance_speed = m_associated_thruster->getAdvanceSpeed();

    float velocity_x_sq = advance_speed * abs(advance_speed) +
                        m_thrust_to_speed_k * m_associated_thruster->getEffort();
    return copysign(sqrt(abs(velocity_x_sq)), velocity_x_sq) * thruster_x;
}

void Rudder::update(Actuators& actuators) {
    // get linear velocity at cp in inertial frame
    auto vel = getFlowVelocity();
    if (vel.Length () <= 0.01) {
        return;
    }

    // pose of body
    auto pose = m_link->WorldPose();

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
    Vector3d link_force = pose.Rot().RotateVectorReverse(world_force);
    actuators.applyForce(m_actuator_id, link_force);
}
