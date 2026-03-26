#include "Wind.hpp"
#include "RockGazeboHelpers.hpp"
#include <gz/msgs/vector3d.pb.h>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Link.hh>

using namespace std;
using namespace gz::sim;
using namespace gazebo_usv;
using namespace gz::math;

Wind::Wind(Wind::EffectParameters const parameters) : m_parameters(parameters) {}

Wind::~Wind()
{
    m_node->Unsubscribe(m_topic_name);
}

void Wind::load(gz::sim::Entity model,
    std::shared_ptr<gz::transport::Node> node,
    sdf::ElementConstPtr const plugin_sdf,
    gz::sim::EntityComponentManager& ecm)
{
    m_model = model;
    m_node = node;
    m_link = rock_gazebo_helpers::resolveLinkWithDefault(model, plugin_sdf, "link_name", ecm);
    gzmsg << "Wind: applying to link " << gz::sim::scopedName(m_link, ecm, "::", false) << endl;

    string topicName = rock_gazebo_helpers::computePluginTopicScope(model, plugin_sdf, ecm) + "/wind";
    m_node->Subscribe(topicName, &Wind::readWindVelocity, this);
    gzmsg << "Wind: receiving wind commands from " << topicName << endl;

    m_parameters = loadParameters(plugin_sdf);
}

Wind::EffectParameters Wind::loadParameters(sdf::ElementConstPtr el) const
{
    gzmsg << "Wind: Loading wind effect parameters" << endl;

    EffectParameters parameters;
    parameters.frontal_area = rock_gazebo_helpers::getParameter<double>("Wind", el, "frontal_area", "m2", 0);
    parameters.lateral_area = rock_gazebo_helpers::getParameter<double>("Wind", el, "lateral_area", "m2", 0);
    parameters.length_overall = rock_gazebo_helpers::getParameter<double>("Wind", el, "length_overall", "m", 0);
    parameters.air_density = rock_gazebo_helpers::getParameter<double>("Wind", el, "air_density", "kg/m3", 1.12);
    parameters.coefficients = rock_gazebo_helpers::getParameter<Vector3d>("Wind", el, "wind_coeffs",
                                                                "", Vector3d(0, 0, 0));
    return parameters;
}

void Wind::readWindVelocity(const gz::msgs::Vector3d& velocity)
{
    m_wind_velocity = Vector3d(velocity.x(), velocity.y(), velocity.z());
}

Wind::Effects Wind::computeEffects(Quaterniond const body2world_orientation, Vector3d const vessel_linear_vel_world, Vector3d const wind_velocity_world) const
{
    Quaterniond world2body_orientation = body2world_orientation.Inverse();
    Vector3d relative_wind_velocity_world = vessel_linear_vel_world - wind_velocity_world;
    Vector3d relative_wind_velocity_body = world2body_orientation * relative_wind_velocity_world;
    relative_wind_velocity_body.Z() = 0;
    if (relative_wind_velocity_body.Length() < 1e-3)
        return Effects{};

    // Compute the wind's angle of attack and its coefficients
    double angle_of_attack_rad = -atan2(relative_wind_velocity_body.Y(), relative_wind_velocity_body.X());
    Angle angle_of_attack(angle_of_attack_rad);
    double wind_coeff_x = -m_parameters.coefficients.X() * cos(angle_of_attack.Radian());
    double wind_coeff_y = m_parameters.coefficients.Y() * sin(angle_of_attack.Radian());
    double wind_coeff_n = m_parameters.coefficients.Z() * sin(2 * angle_of_attack.Radian());

    // Compute wind effects for X, Y and N
    Effects wind_effects;
    wind_effects.force[0] = 0.5 * m_parameters.air_density * relative_wind_velocity_body.SquaredLength() * wind_coeff_x * m_parameters.frontal_area;
    wind_effects.force[1] = 0.5 * m_parameters.air_density * relative_wind_velocity_body.SquaredLength() * wind_coeff_y * m_parameters.lateral_area;
    wind_effects.torque[2] = 0.5 * m_parameters.air_density * relative_wind_velocity_body.SquaredLength() * wind_coeff_n * m_parameters.lateral_area * m_parameters.length_overall;
    return wind_effects;
}

void Wind::update(gz::sim::EntityComponentManager& ecm)
{
    Link link(m_link);
    auto body2world_pose = link.WorldPose(ecm).value();
    auto body2world_q = body2world_pose.Rot();

    // Compute the new force and torque for this timestep
    Effects effects = computeEffects(body2world_q, link.WorldLinearVelocity(ecm).value(), m_wind_velocity);

    // Apply force and torque
    auto world_force = body2world_q * effects.force;
    auto world_torque = body2world_q * effects.torque;
    link.AddWorldWrench(ecm, world_force, world_torque);
}