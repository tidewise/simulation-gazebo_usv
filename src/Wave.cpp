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
    string topicName = utilities::getNamespaceFromPluginName(pluginName);
    
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

Wave::Effects Wave::computeEffects(double seconds, Vector3d const wave_amplitude_world, Vector3d const wave_frequency_world) const
{
    Vector3d plane_wave_amplitude  = wave_amplitude_world;
    plane_wave_amplitude.Z() = 0;

    // Compute the wave's time coefficients
    double wave_coeff_time_x = sin(M_PI*2*seconds*wave_frequency_world.X() + phase_x);
    double wave_coeff_time_y = sin(M_PI*2*seconds*wave_frequency_world.Y() + phase_y);
    double wave_coeff_time_z = sin(M_PI*2*seconds*wave_frequency_world.Z() + phase_z);

    // Compute wave effects for X, Y and N
    Effects wave_effects;
    wave_effects.force[0] = wave_coeff_time_x * wave_amplitude_world.X();
    wave_effects.force[1] = wave_coeff_time_y * wave_amplitude_world.Y();
    wave_effects.force[2] = wave_coeff_time_z * wave_amplitude_world.Z();
    wave_effects.torque[0] = (wave_coeff_time_y+wave_coeff_time_x) * plane_wave_amplitude.Length() * mParameters.torque_constant;
    return wave_effects;
}

void Wave::update()
{
    // Compute the new force and torque for this timestep
    base::Time current_time = base::Time::now();

    Effects effects = computeEffects(current_time.toSeconds(), mWaveAmplitude, mWaveFrequency);

    // Apply force and torque
    mLink->AddRelativeForce(effects.force);
    mLink->AddRelativeTorque(effects.torque);
}

