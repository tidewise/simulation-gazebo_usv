#include "Wind.hpp"
#include "Utilities.hpp"

using namespace std;
using namespace gazebo;
using namespace gazebo_usv;
using namespace ignition::math;

Wind::Wind(Wind::EffectParameters const parameters) : m_parameters(parameters) {}

Wind::~Wind()
{
    if (m_wind_velocity_subscriber)
    {
        m_wind_velocity_subscriber->Unsubscribe();
    }
}

void Wind::load(ModelPtr const _model, transport::NodePtr const _node, sdf::ElementPtr const _sdf)
{
    m_model = _model;
    m_node = _node;
    m_link = getReferenceLink(m_model, _sdf);

    auto pluginName = _sdf->Get<std::string>("name");
    string topicName = utilities::getNamespaceFromPluginName(pluginName) + "/wind_velocity";
    if (m_wind_velocity_subscriber)
    {
        m_wind_velocity_subscriber->Unsubscribe();
    }
    m_wind_velocity_subscriber = m_node->Subscribe("/" + topicName, &Wind::readWindVelocity, this);
    auto worldName = m_model->GetWorld()->Name();
    gzmsg << "Wind: receiving wind commands from /"
          << topicName << endl;

    m_parameters = loadParameters(_sdf);
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
    gzmsg << "Wind: Loading wind effect parameters" << endl;

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
    m_wind_velocity = Vector3d(velocity->x(), velocity->y(), velocity->z());
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

void Wind::update()
{
    // Compute the new force and torque for this timestep 
    Effects effects = computeEffects(m_model->WorldPose().Rot(), m_model->WorldLinearVel(), m_wind_velocity);

    // Apply force and torque
    m_link->AddRelativeForce(effects.force);
    m_link->AddRelativeTorque(effects.torque);
}