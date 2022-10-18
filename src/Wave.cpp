#include "Wave.hpp"
#include "Utilities.hpp"
#include <cstdlib> 
#include <math.h>

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;
using namespace ignition::math;

Wave::Wave(Wave::EffectParameters const parameters) : mParameters(parameters) {}

Wave::~Wave()
{
    if (mWaveAmplitudeSubscriber)
    {
        mWaveAmplitudeSubscriber->Unsubscribe();
    }
    if (mWaveFrequencySubscriber)
    {
        mWaveFrequencySubscriber->Unsubscribe();
    }
}

void Wave::load(ModelPtr const _model, transport::NodePtr const _node, sdf::ElementPtr const _sdf)
{
    mModel = _model;
    mNode = _node;
    mLink = getReferenceLink(mModel, _sdf);
    
    auto pluginName = _sdf->Get<std::string>("name");
    string topicName = utilities::getTopicNameFromPluginName(pluginName);
    
    string topicNameAmplitude = topicName + "/wave_amplitude";


    if (mWaveAmplitudeSubscriber)
    {
        mWaveAmplitudeSubscriber->Unsubscribe();
    }
    mWaveAmplitudeSubscriber = mNode->Subscribe("/" + topicNameAmplitude, &Wave::readWaveAmplitude, this);
    
    string topicNameFrequency = topicName + "/wave_frequency";
    if (mWaveFrequencySubscriber)
    {
        mWaveFrequencySubscriber->Unsubscribe();
    }
    mWaveFrequencySubscriber = mNode->Subscribe("/" + topicNameFrequency, &Wave::readWaveFrequency, this);
    
    auto worldName = mModel->GetWorld()->Name();
    gzmsg << "Wave: receiving wave commands from /"
          << topicNameAmplitude << "and /" << topicNameFrequency << endl;

    mParameters = loadParameters(_sdf);
    std::srand(static_cast<unsigned int>(time(NULL)));
    phase_x = M_PI * (double) rand()/RAND_MAX;
    phase_y = M_PI * (double) rand()/RAND_MAX;
    phase_z = M_PI * (double) rand()/RAND_MAX;

}

physics::LinkPtr Wave::getReferenceLink(physics::ModelPtr const _model, sdf::ElementPtr const _sdf) const
{
    if (_sdf->HasElement("link_name"))
    {
        physics::LinkPtr link = _model->GetLink(_sdf->Get<string>("link_name"));
        if (!link)
        {
            string msg = "Wave: link " + _sdf->Get<string>("link_name") + " not found in model " + _model->GetName();
            gzthrow(msg.c_str());
        }
        gzmsg << "Wave: reference link: " << link->GetName() << endl;
        return link;
    }
    else if (_model->GetLinks().empty())
    {
        gzthrow("Wave: no link was defined in model!");
    }
    else
    {
        physics::LinkPtr link = _model->GetLinks().front();
        gzmsg << "Wave: reference link not defined, using " << link->GetName() << " instead!" << endl;
        return link;
    }
}

Wave::EffectParameters Wave::loadParameters(sdf::ElementPtr el) const
{
    gzmsg << "Wave: Loading wave effect parameters" << endl;

    EffectParameters parameters;
    parameters.torque_constant = utilities::getParameter<double>("Wave", el, "torque_constant", "", 0);
    return parameters;
}

void Wave::readWaveAmplitude(const ConstVector3dPtr &amplitude)
{
    mWaveAmplitude = Vector3d(amplitude->x(), amplitude->y(), amplitude->z());
}

void Wave::readWaveFrequency(const ConstVector3dPtr &frequency)
{
    mWaveFrequency = Vector3d(frequency->x(), frequency->y(), frequency->z());
}

Wave::Effects Wave::computeEffects(double seconds, Quaterniond const body2world_orientation, Vector3d const vessel_linear_vel_world, Vector3d const wave_amplitude_world, Vector3d const wave_frequency_world) const
{
    // Compute the wave's time coefficients
    double wave_coeff_time_x = sin(M_PI*2*seconds*wave_frequency_world.X() + phase_x);
    double wave_coeff_time_y = sin(M_PI*2*seconds*wave_frequency_world.Y() + phase_y);
    double wave_coeff_time_z = sin(M_PI*2*seconds*wave_frequency_world.Z() + phase_z);

    Quaterniond world2body_orientation = body2world_orientation.Inverse();
    Vector3d current_wave_amplitude  = wave_amplitude_world * Vector3d(wave_coeff_time_x,wave_coeff_time_y,wave_coeff_time_z);

    Vector3d relative_wave_amplitude_world = vessel_linear_vel_world - current_wave_amplitude;
    Vector3d relative_wave_amplitude_body = world2body_orientation * relative_wave_amplitude_world;

    if (relative_wave_amplitude_body.Length() < 1e-3)
        return Effects{};
    relative_wave_amplitude_body.Z() = 0;

    // Compute the wave's angle of attack and its coefficients
    double angle_of_attack_rad = -atan2(relative_wave_amplitude_body.Y(), relative_wave_amplitude_body.X());
    Angle angle_of_attack(angle_of_attack_rad);
    double wave_coeff_x = - cos(angle_of_attack.Radian());
    double wave_coeff_y = sin(angle_of_attack.Radian());
    double wave_coeff_z = wave_coeff_time_z;
    double wave_coeff_n = mParameters.torque_constant *sin(2 * angle_of_attack.Radian());

    // Compute wave effects for X, Y and N
    Effects wave_effects;
    wave_effects.force[0] = wave_coeff_x * relative_wave_amplitude_body.Length();
    wave_effects.force[1] = wave_coeff_y * relative_wave_amplitude_body.Length();
    wave_effects.force[2] = wave_coeff_z * wave_amplitude_world.Z();
    wave_effects.torque[0] =  wave_coeff_n * relative_wave_amplitude_body.Length();
    return wave_effects;
}

void Wave::update()
{
    // Compute the new force and torque for this timestep
    base::Time current_time = base::Time::now();

    Effects effects = computeEffects(current_time.toSeconds(),mModel->WorldPose().Rot(), mModel->WorldLinearVel(), mWaveAmplitude, mWaveFrequency);

    // Apply force and torque
    mLink->AddRelativeForce(effects.force);
    mLink->AddRelativeTorque(effects.torque);
}
