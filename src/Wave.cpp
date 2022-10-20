#include "Wave.hpp"
#include "Utilities.hpp"
#include <cstdlib>
#include <math.h>

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;
using namespace ignition::math;

Wave::Wave(Wave::EffectParameters const parameters)
    : m_parameters(parameters)
{
}

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

void Wave::load(ModelPtr const _model,
    transport::NodePtr const _node,
    sdf::ElementPtr const _sdf)
{
    m_model = _model;
    m_node = _node;
    m_link = getReferenceLink(m_model, _sdf);

    auto pluginName = _sdf->Get<std::string>("name");
    string topicName = utilities::getNamespaceFromPluginName(pluginName);

    string topicNameAmplitude = topicName + "/wave_amplitude";
    if (m_wave_amplitude_subscriber) {
        m_wave_amplitude_subscriber->Unsubscribe();
    }
    m_wave_amplitude_subscriber =
        m_node->Subscribe("/" + topicNameAmplitude, &Wave::readWaveAmplitude, this);

    string topicNameFrequency = topicName + "/wave_frequency";
    if (m_wave_frequency_subscriber) {
        m_wave_frequency_subscriber->Unsubscribe();
    }
    m_wave_frequency_subscriber =
        m_node->Subscribe("/" + topicNameFrequency, &Wave::readWaveFrequency, this);

    string topicNameRoll = topicName + "/roll_vector";
    if (m_roll_subscriber) {
        m_roll_subscriber->Unsubscribe();
    }
    m_roll_subscriber = m_node->Subscribe("/" + topicNameRoll, &Wave::readRoll, this);

    auto worldName = m_model->GetWorld()->Name();
    gzmsg << "Wave: receiving wave commands from /" << topicNameAmplitude << "and /"
          << topicNameFrequency << endl;

    m_parameters = loadParameters(_sdf);
    std::srand(static_cast<unsigned int>(time(NULL)));
    m_phase_x = M_PI * (double)rand() / RAND_MAX;
    m_phase_y = M_PI * (double)rand() / RAND_MAX;
    m_phase_z = M_PI * (double)rand() / RAND_MAX;
    m_phase_n = M_PI * (double)rand() / RAND_MAX;
}

physics::LinkPtr Wave::getReferenceLink(physics::ModelPtr const _model,
    sdf::ElementPtr const _sdf) const
{
    if (_sdf->HasElement("link_name")) {
        physics::LinkPtr link = _model->GetLink(_sdf->Get<string>("link_name"));
        if (!link) {
            string msg = "Wave: link " + _sdf->Get<string>("link_name") +
                         " not found in model " + _model->GetName();
            gzthrow(msg.c_str());
        }
        gzmsg << "Wave: reference link: " << link->GetName() << endl;
        return link;
    }
    else if (_model->GetLinks().empty()) {
        gzthrow("Wave: no link was defined in model!");
    }
    else {
        physics::LinkPtr link = _model->GetLinks().front();
        gzmsg << "Wave: reference link not defined, using " << link->GetName()
              << " instead!" << endl;
        return link;
    }
}

Wave::EffectParameters Wave::loadParameters(sdf::ElementPtr el) const
{
    gzmsg << "Wave: Loading wave effect parameters" << endl;

    EffectParameters parameters;
    parameters.torque_constant =
        utilities::getParameter<double>("Wave", el, "torque_constant", "", 0);
    return parameters;
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
        wave_torque_scale_n * roll_amplitude_world * m_parameters.torque_constant;
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
