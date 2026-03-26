#include "Wave.hpp"
#include "RockGazeboHelpers.hpp"
#include <cstdlib>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Link.hh>
#include <math.h>

using namespace std;
using namespace gz::sim;
using namespace gazebo_usv;
using namespace gz::math;

Wave::~Wave()
{
    m_node->Unsubscribe(m_topic_name);
}

void Wave::load(gz::sim::Entity model,
    std::shared_ptr<gz::transport::Node> node,
    sdf::ElementConstPtr const sdf,
    gz::sim::EntityComponentManager& ecm)
{
    m_model = model;
    m_node = node;
    m_link = rock_gazebo_helpers::resolveLinkWithDefault(m_model,
        sdf,
        "link_name",
        ecm);
    gzmsg << "Wave: applying to link " << gz::sim::scopedName(m_link, ecm, "::", false) << std::endl;

    string topicScope = rock_gazebo_helpers::computePluginTopicScope(model, sdf, ecm);
    m_topic_name = topicScope + "/waves";
    m_node->Subscribe(m_topic_name, &Wave::read, this);

    gzmsg << "Wave: receiving wave commands from " << m_topic_name << endl;

    std::srand(static_cast<unsigned int>(time(NULL)));
    m_phase_x = M_PI * (double)rand() / RAND_MAX;
    m_phase_y = M_PI * (double)rand() / RAND_MAX;
    m_phase_z = M_PI * (double)rand() / RAND_MAX;
    m_phase_n = M_PI * (double)rand() / RAND_MAX;
}

void Wave::read(gz::gazebo_usv::Wave const& wave)
{
    m_wave_amplitude = rock_gazebo_helpers::proto2Gz(wave.amplitude());
    m_wave_frequency = rock_gazebo_helpers::proto2Gz(wave.frequency());
    m_roll_amplitude = wave.roll_amplitude();
    m_roll_frequency = wave.roll_frequency();
}

Wave::Effects Wave::computeEffects(double seconds,
    Vector3d const wave_amplitude_world,
    Vector3d const wave_frequency_world,
    double roll_amplitude_world,
    double roll_frequency_world) const
{
    Vector3d plane_wave_amplitude = wave_amplitude_world;
    plane_wave_amplitude.Z() = 0;

    // Compute the wave's time coefficients
    double wave_force_scale_x =
        sin(M_PI * 2 * seconds * wave_frequency_world.X() + m_phase_x);
    double wave_force_scale_y =
        sin(M_PI * 2 * seconds * wave_frequency_world.Y() + m_phase_y);
    double wave_force_scale_z =
        sin(M_PI * 2 * seconds * wave_frequency_world.Z() + m_phase_z);
    double wave_torque_scale_n =
        sin(M_PI * 2 * seconds * roll_frequency_world + m_phase_n);

    // Compute wave effects for X, Y and N
    Effects wave_effects;
    wave_effects.force[0] = wave_force_scale_x * wave_amplitude_world.X();
    wave_effects.force[1] = wave_force_scale_y * wave_amplitude_world.Y();
    wave_effects.force[2] = wave_force_scale_z * wave_amplitude_world.Z();
    wave_effects.torque[0] = wave_torque_scale_n * roll_amplitude_world;
    return wave_effects;
}

void Wave::update(gz::sim::EntityComponentManager& ecm)
{
    // Compute the new force and torque for this timestep
    base::Time current_time = base::Time::now();

    Effects effects = computeEffects(current_time.toSeconds(),
        m_wave_amplitude,
        m_wave_frequency,
        m_roll_amplitude,
        m_roll_frequency);

    // Apply force and torque
    auto link2world = Link(m_link).WorldPose(ecm);
    auto link2world_q = link2world->Rot();

    auto world_force = link2world_q * effects.force;
    auto world_torque = link2world_q * effects.torque;
    Link(m_link).AddWorldWrench(ecm, world_force, world_torque);
}
