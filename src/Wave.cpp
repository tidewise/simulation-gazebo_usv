#include "Wave.hpp"
#include "RockGazeboHelpers.hpp"
#include <cstdlib>
#include <math.h>

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;
using namespace ignition::math;


Wave::~Wave()
{
    if (m_wave_amplitude_subscriber) {
        m_wave_amplitude_subscriber->Unsubscribe();
    }
    if (m_wave_frequency_subscriber) {
        m_wave_frequency_subscriber->Unsubscribe();
    }
    if (m_roll_subscriber) {
        m_roll_subscriber->Unsubscribe();
    }
}

void Wave::load(ModelPtr const model,
    transport::NodePtr const node,
    sdf::ElementPtr const plugin_sdf)
{
    m_model = model;
    m_node = node;
    m_link = rock_gazebo_helpers::resolveLinkWithDefault(m_model, plugin_sdf, "link_name");
    gzmsg << "Wave: applying to link " << m_link->GetScopedName() << std::endl;

    string topicScope = rock_gazebo_helpers::computePluginTopicScope(model, plugin_sdf);

    string topicNameAmplitude = topicScope + "/wave_amplitude";
    if (m_wave_amplitude_subscriber) {
        m_wave_amplitude_subscriber->Unsubscribe();
    }
    m_wave_amplitude_subscriber =
        m_node->Subscribe(topicNameAmplitude, &Wave::readWaveAmplitude, this);

    string topicNameFrequency = topicScope + "/wave_frequency";
    if (m_wave_frequency_subscriber) {
        m_wave_frequency_subscriber->Unsubscribe();
    }
    m_wave_frequency_subscriber =
        m_node->Subscribe(topicNameFrequency, &Wave::readWaveFrequency, this);

    string topicNameRoll = topicScope + "/roll_vector";
    if (m_roll_subscriber) {
        m_roll_subscriber->Unsubscribe();
    }
    m_roll_subscriber = m_node->Subscribe(topicNameRoll, &Wave::readRoll, this);

    auto worldName = m_model->GetWorld()->Name();
    gzmsg
        << "Wave: receiving wave commands from "
        << topicNameAmplitude << ", " << topicNameFrequency
        << " and " << topicNameRoll << endl;

    std::srand(static_cast<unsigned int>(time(NULL)));
    m_phase_x = M_PI * (double)rand() / RAND_MAX;
    m_phase_y = M_PI * (double)rand() / RAND_MAX;
    m_phase_z = M_PI * (double)rand() / RAND_MAX;
    m_phase_n = M_PI * (double)rand() / RAND_MAX;
}

void Wave::readWaveAmplitude(const ConstVector3dPtr& amplitude)
{
    m_wave_amplitude = Vector3d(amplitude->x(), amplitude->y(), amplitude->z());
}

void Wave::readWaveFrequency(const ConstVector3dPtr& frequency)
{
    m_wave_frequency = Vector3d(frequency->x(), frequency->y(), frequency->z());
}

void Wave::readRoll(const ConstVector2dPtr& roll)
{
    m_roll_amplitude = roll->x();
    m_roll_frequency = roll->y();
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
    wave_effects.torque[0] =
        wave_torque_scale_n * roll_amplitude_world;
    return wave_effects;
}

void Wave::update()
{
    // Compute the new force and torque for this timestep
    base::Time current_time = base::Time::now();

    Effects effects = computeEffects(current_time.toSeconds(),
        m_wave_amplitude,
        m_wave_frequency,
        m_roll_amplitude,
        m_roll_frequency);

    // Apply force and torque
    m_link->AddRelativeForce(effects.force);
    m_link->AddRelativeTorque(effects.torque);
}
