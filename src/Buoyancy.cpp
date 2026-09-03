#include <gazebo_usv/Buoyancy.hpp>
#include <gazebo_usv/Components.hpp>
#include <gazebo_usv/RockGazeboHelpers.hpp>
#include <gazebo_usv/USVPlugin.hpp>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>

#include <algorithm>

using namespace gazebo_usv;
using namespace std;
using namespace gz::sim;
using namespace gazebo_usv;
using namespace gz::math;

Buoyancy::Buoyancy(USVPlugin& plugin,
    gz::sim::Entity model,
    sdf::ElementConstPtr buoyancy_sdf,
    gz::sim::EntityComponentManager& ecm,
    double default_water_level)
{
    m_link_name = buoyancy_sdf->Get<string>("name");
    m_link = rock_gazebo_helpers::resolveLinkRecursive(model, m_link_name, ecm);
    Link(m_link).EnableBoundingBoxChecks(ecm);

    m_default_water_level = default_water_level;
    m_buoyancy_force = buoyancy_sdf->Get<double>("buoyancy_force", 0).first;
    m_center_of_buoyancy =
        buoyancy_sdf->Get<Vector3d>("center_of_buoyancy", Vector3d::Zero).first;
}

void Buoyancy::update(gz::sim::EntityComponentManager& ecm)
{
    auto submersed_ratio = calculateSubmersedRatio(ecm);
    if (!submersed_ratio.has_value()) {
        return;
    }

    auto buoyancy = Vector3d(0, 0, submersed_ratio.value() * m_buoyancy_force);
    Link(m_link).AddWorldForce(ecm, buoyancy, m_center_of_buoyancy);
}

optional<double> Buoyancy::calculateSubmersedRatio(
    gz::sim::EntityComponentManager& ecm) const
{
    auto bb = Link(m_link).WorldAxisAlignedBox(ecm);
    if (!bb.has_value()) {
        return {};
    }

    optional<double> link_water_level = ecm.ComponentData<gazebo_usv::WaterLevel>(m_link);
    double water_level = m_default_water_level;
    if (link_water_level.has_value()) {
        water_level = link_water_level.value();
    }

    // Distance of the lower part of the bounding box to the surface
    // It is positive when submerged
    double distance_to_surface = water_level - bb->Min().Z();
    double submersed_ratio = distance_to_surface / bb->ZLength();

    return std::clamp<double>(submersed_ratio, 0, 1);
}
