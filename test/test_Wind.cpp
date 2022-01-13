#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
#include <gazebo_usv/Wind.hpp>

using namespace std;
using namespace ignition::math;
using namespace gazebo_usv;
namespace gaz = gazebo::physics;

struct WindTest : public ::testing::Test
{
    typedef Wind::EffectParameters Params;

    Params parameters;
    Vector3d wind_velocity;
    Vector3d vessel_linear_vel;
    Pose3d vessel_world_pose;

    WindTest()
    {
        parameters.frontal_area = 10;
        parameters.lateral_area = 25;
        parameters.length_overall = 5;
        parameters.air_density = 1.269;
        parameters.coefficients = Vector3d({0.5, 0.8, 0.1});
        vessel_world_pose = Pose3d(Vector3d(), Quaterniond());
    }

    Wind plugin()
    {
        return Wind(parameters);
    }
};

TEST_F(WindTest, it_applies_force_in_the_world_x_axis_when_perpendicular_to_the_unactuated_vessel)
{
    vessel_linear_vel = Vector3d({0, 0, 0});
    wind_velocity = Vector3d({1, 0, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, -IGN_PI / 2}));
    auto plugin = this->plugin();

    auto effects = plugin.computeEffects(vessel_world_pose.Rot(), vessel_linear_vel, wind_velocity);
    auto forces_world = vessel_world_pose.Rot() * effects.force;
    ASSERT_TRUE(forces_world.X() > 0);
    ASSERT_NEAR(forces_world.Y(), 0, 0.001);
    ASSERT_NEAR(forces_world.Z(), 0, 0.001);
}

TEST_F(WindTest, it_applies_negative_force_in_world_x_positive_force_in_y_and_no_torque_when_the_wind_is_perpendicular_to_an_unactuated_vessel)
{
    vessel_linear_vel = Vector3d({0, 0, 0});
    wind_velocity = Vector3d({-1, 1, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, IGN_PI / 4}));
    auto plugin = this->plugin();

    auto effects = plugin.computeEffects(vessel_world_pose.Rot(), vessel_linear_vel, wind_velocity);
    auto force_world = vessel_world_pose.Rot() * effects.force;
    ASSERT_TRUE(force_world.X() < 0);
    ASSERT_TRUE(force_world.Y() > 0);
    ASSERT_NEAR(effects.torque.Z(), 0, 0.001);
}

TEST_F(WindTest, it_applies_negative_force_in_world_x_axis_positive_force_in_y_axis_and_positive_torque_when_not_perpendicular_to_an_unactuated_vessel)
{
    vessel_linear_vel = Vector3d({0, 0, 0});
    wind_velocity = Vector3d({-1, 0, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, IGN_PI / 4}));
    auto plugin = this->plugin();

    auto effects = plugin.computeEffects(vessel_world_pose.Rot(), vessel_linear_vel, wind_velocity);
    auto force_world = vessel_world_pose.Rot() * effects.force;
    ASSERT_TRUE(force_world.X() < 0);
    ASSERT_TRUE(force_world.Y() > 0);
    ASSERT_TRUE(effects.torque.Z() > 0);
}

TEST_F(WindTest, it_applies_positive_y_force_in_body_frame_when_the_apparent_wind_is_perpendicular_to_the_vessel)
{
    vessel_linear_vel = Vector3d({1, 0, 0});
    wind_velocity = Vector3d({1, 1, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, 0}));
    auto plugin = this->plugin();

    auto effects = plugin.computeEffects(vessel_world_pose.Rot(), vessel_linear_vel, wind_velocity);
    ASSERT_NEAR(effects.force.X(), 0, 0.001);
    ASSERT_TRUE(effects.force.Y() > 0);
    ASSERT_NEAR(effects.torque.Z(), 0, 0.001);
}

TEST_F(WindTest, it_applies_negative_x_force_in_body_frame_when_the_apparent_wind_is_180_degrees_to_the_vessel)
{
    vessel_linear_vel = Vector3d({1, 0, 0});
    wind_velocity = Vector3d({-1, 0, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, 0}));
    auto plugin = this->plugin();

    auto effects = plugin.computeEffects(vessel_world_pose.Rot(), vessel_linear_vel, wind_velocity);
    ASSERT_TRUE(effects.force.X() < 0);
    ASSERT_NEAR(effects.force.Y(), 0, 0.001);
    ASSERT_NEAR(effects.torque.Z(), 0, 0.001);
}

TEST_F(WindTest, it_applies_positive_y_force_negative_x_force_and_positive_torque_in_body_frame_when_the_apparent_wind_is_negative_45_degrees_to_the_vessel)
{
    vessel_linear_vel = Vector3d({1, 0, 0});
    wind_velocity = Vector3d({-1, 2, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, 0}));
    auto plugin = this->plugin();

    auto effects = plugin.computeEffects(vessel_world_pose.Rot(), vessel_linear_vel, wind_velocity);
    ASSERT_TRUE(effects.force.X() < 0);
    ASSERT_TRUE(effects.force.Y() > 0);
    ASSERT_TRUE(effects.torque.Z() > 0);
}

TEST_F(WindTest, it_applies_neither_force_nor_torque_when_the_apparent_wind_is_zero)
{
    vessel_linear_vel = Vector3d({1, 1, 0});
    wind_velocity = Vector3d({1, 1, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, IGN_PI/4}));
    auto plugin = this->plugin();

    auto effects = plugin.computeEffects(vessel_world_pose.Rot(), vessel_linear_vel, wind_velocity);
    ASSERT_NEAR(effects.force.X(), 0, 0.001);
    ASSERT_NEAR(effects.force.Y(), 0, 0.001);
    ASSERT_NEAR(effects.torque.Z(), 0, 0.001);
}