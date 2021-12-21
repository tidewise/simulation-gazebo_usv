#include "Wind.hpp"
#include "Utilities.hpp"

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;
using namespace ignition::math;

Wind::~Wind()
{
    if (mWindVelocitySubscriber)
    {
        mWindVelocitySubscriber->Unsubscribe();
    }
}

void Wind::load(ModelPtr const _model, transport::NodePtr const _node, sdf::ElementPtr const _sdf)
{
    mModel = _model;
    mNode = _node;
    mLink = getReferenceLink(mModel, _sdf);

    string topicName = mModel->GetName() + "/wind_velocity";
    if (mWindVelocitySubscriber)
    {
        mWindVelocitySubscriber->Unsubscribe();
    }
    mWindVelocitySubscriber = mNode->Subscribe("~/" + topicName, &Wind::readWindVelocity, this);
    auto worldName = mModel->GetWorld()->Name();
    gzmsg << "Wind: receiving wind commands from /gazebo/"
          << worldName << "/" << topicName << endl;

    mParameters = loadParameters(_sdf);
}

physics::LinkPtr Wind::getReferenceLink(physics::ModelPtr const _model, sdf::ElementPtr const _sdf) const
{
    if (_sdf->HasElement("link_name"))
    {
        physics::LinkPtr link = _model->GetLink(_sdf->Get<string>("link_name"));
        if (!link)
        {
            string msg = "Wind: link " + _sdf->Get<string>("link_name") + " not found in model " + _model->GetName();
            gzthrow(msg.c_str());
        }
        gzmsg << "Wind: reference link: " << link->GetName() << endl;
        return link;
    }
    else if (_model->GetLinks().empty())
    {
        gzthrow("Wind: no link was defined in model!");
    }
    else
    {
        physics::LinkPtr link = _model->GetLinks().front();
        gzmsg << "Wind: reference link not defined, using " << link->GetName() << " instead!" << endl;
        return link;
    }
}

Wind::EffectParameters Wind::loadParameters(sdf::ElementPtr el) const
{
    gzmsg << "Loading wind effect parameters" << endl;

    EffectParameters parameters;
    parameters.frontal_area = utilities::getParameter<double>("Wind", el, "frontal_area", "m2", 0);
    parameters.lateral_area = utilities::getParameter<double>("Wind", el, "lateral_area", "m2", 0);
    parameters.length_overall = utilities::getParameter<double>("Wind", el, "length_overall", "m", 0);
    parameters.air_density = utilities::getParameter<double>("Wind", el, "air_density", "kg/m3", 1.12);
    parameters.coefficients = utilities::getParameter<Vector3d>("Wind", el, "wind_coeffs",
                                                                "", Vector3d(0, 0, 0));
    return parameters;
}

void Wind::readWindVelocity(const ConstVector3dPtr &velocity)
{
    mWindVelocity = Vector3d(velocity->x(), velocity->y(), velocity->z());
}

Wind::Effects Wind::computeEffects()
{
    // Compute relative velocities and angle of attack
    Angle model_yaw = mModel->WorldPose().Rot().Euler().Z();
    Angle wind_direction(atan2(-mWindVelocity.Y(), mWindVelocity.X()));
    Angle wind_angle_from_bf(wind_direction - model_yaw);                                                                                               // Wind angle from body frame
    Vector3d wind_velocity_bf(mWindVelocity.Length() * cos(wind_angle_from_bf.Radian()), mWindVelocity.Length() * sin(wind_angle_from_bf.Radian()), 0); // Wind velocity in the body frame
    Vector3d relative_velocity = mModel->WorldLinearVel() - wind_velocity_bf;
    if (!relative_velocity.X() && !relative_velocity.Y())
        return Effects{};
    // Angle of attack in rad
    double angle_of_attack_rad = -atan2(relative_velocity.Y(), relative_velocity.X());
    Angle angle_of_attack(-angle_of_attack_rad);
    double relative_velocity_squared = pow(relative_velocity.Length(), 2);

    // Compute wind coefficients
    double wind_coeff_x = -mParameters.coefficients.X() * cos(angle_of_attack.Radian());
    double wind_coeff_y = mParameters.coefficients.Y() * sin(angle_of_attack.Radian());
    double wind_coeff_n = mParameters.coefficients.Z() * sin(2 * angle_of_attack.Radian());

    // Compute wind effects for X, Y and N
    Effects wind_effects;
    wind_effects.force[0] = 0.5 * mParameters.air_density * relative_velocity_squared * wind_coeff_x * mParameters.frontal_area;
    wind_effects.force[1] = 0.5 * mParameters.air_density * relative_velocity_squared * wind_coeff_y * mParameters.lateral_area;
    wind_effects.torque[2] = 0.5 * mParameters.air_density * relative_velocity_squared * wind_coeff_n * mParameters.lateral_area * mParameters.length_overall;
    return wind_effects;
}

void Wind::update()
{

    // Compute the new force and torque for this timestep
    Effects effects = computeEffects();
    mLink->AddRelativeForce(effects.force);
    mLink->AddRelativeTorque(effects.torque);
}