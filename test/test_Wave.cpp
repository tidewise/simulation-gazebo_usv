#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
#include <gazebo_usv/Wave.hpp>
#include "base/Time.hpp"
#include <math.h>

using namespace std;
using namespace ignition::math;
using namespace gazebo_usv;
namespace gaz = gazebo::physics;

struct WaveTest : public ::testing::Test
{
    typedef Wave::EffectParameters Params;

    Params parameters;
    Vector3d wave_amplitude;
    Vector3d wave_frequency;
    Vector3d vessel_linear_vel;
    Pose3d vessel_world_pose;

    WaveTest()
    {
        parameters.torque_constant = 0.4;
    }

    Wave plugin()
    {
        return Wave(parameters);
    }
    
};

TEST_F(WaveTest, it_applies_force_in_the_world_x_axis_when_perpendicular_to_the_unactuated_vessel)
{
    vessel_linear_vel = Vector3d({0, 0, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, -IGN_PI / 2}));
    wave_amplitude = Vector3d({1, 0, 0});
    wave_frequency = Vector3d({0.3, 0, 0});
    auto plugin = this->plugin();
    plugin.phase_x = M_PI/2;
    plugin.phase_y = M_PI/2;
    plugin.phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time,vessel_world_pose.Rot(), vessel_linear_vel, wave_amplitude,wave_frequency);
    auto forces_world = vessel_world_pose.Rot() * effects.force;
    ASSERT_TRUE(forces_world.X() > 0);
    ASSERT_NEAR(forces_world.Y(), 0, 0.001);
    ASSERT_NEAR(forces_world.Z(), 0, 0.001);
}

TEST_F(WaveTest, it_applies_force_in_the_world_x_axis_when_perpendicular_to_the_unactuated_vessel_at_different_time)
{
    vessel_linear_vel = Vector3d({0, 0, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, -IGN_PI / 2}));
    wave_amplitude = Vector3d({1, 0, 0});
    wave_frequency = Vector3d({0.3, 0, 0});
    auto plugin = this->plugin();
    plugin.phase_x = M_PI/2;
    plugin.phase_y = M_PI/2;
    plugin.phase_z = M_PI/2;
    double current_time = M_PI;

    auto effects = plugin.computeEffects(current_time,vessel_world_pose.Rot(), vessel_linear_vel, wave_amplitude,wave_frequency);
    auto forces_world = vessel_world_pose.Rot() * effects.force;
    ASSERT_TRUE(forces_world.X() > 0);
    ASSERT_NEAR(forces_world.Y(), 0, 0.001);
    ASSERT_NEAR(forces_world.Z(), 0, 0.001);
}

TEST_F(WaveTest, it_applies_force_in_the_world_z_axis_to_the_unactuated_vessel)
{
    vessel_linear_vel = Vector3d({0, 0, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, -IGN_PI / 2}));
    wave_amplitude = Vector3d({0, 0, 1});
    wave_frequency = Vector3d({0, 0, 0.3});
    auto plugin = this->plugin();
    plugin.phase_x = M_PI/2;
    plugin.phase_y = M_PI/2;
    plugin.phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time,vessel_world_pose.Rot(), vessel_linear_vel, wave_amplitude,wave_frequency);
    auto forces_world = vessel_world_pose.Rot() * effects.force;
    ASSERT_TRUE(forces_world.Z() > 0);
    ASSERT_NEAR(forces_world.X(), 0, 0.001);
    ASSERT_NEAR(forces_world.Y(), 0, 0.001);
}

TEST_F(WaveTest, it_applies_negative_force_in_world_x_positive_force_in_y_and_no_torque_when_the_wave_is_perpendicular_to_an_unactuated_vessel)
{
    vessel_linear_vel = Vector3d({0, 0, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, IGN_PI / 4}));
    wave_amplitude = Vector3d({-1, 1, 0});
    wave_frequency = Vector3d({0.3, 0.3, 0});
    auto plugin = this->plugin();
    plugin.phase_x = M_PI/2;
    plugin.phase_y = M_PI/2;
    plugin.phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time,vessel_world_pose.Rot(), vessel_linear_vel, wave_amplitude,wave_frequency);
    auto forces_world = vessel_world_pose.Rot() * effects.force;
    ASSERT_TRUE(forces_world.X() < 0);
    ASSERT_TRUE(forces_world.Y() > 0);
    ASSERT_NEAR(effects.torque.X(), 0, 0.001);
}

TEST_F(WaveTest, it_applies_negative_force_in_world_x_axis_positive_force_in_y_axis_and_positive_torque_when_not_perpendicular_to_an_unactuated_vessel)
{
    vessel_linear_vel = Vector3d({0, 0, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, IGN_PI / 4}));
    wave_amplitude = Vector3d({-1, 0, 0});
    wave_frequency = Vector3d({0.3, 0, 0});
    auto plugin = this->plugin();
    plugin.phase_x = M_PI/2;
    plugin.phase_y = M_PI/2;
    plugin.phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time,vessel_world_pose.Rot(), vessel_linear_vel, wave_amplitude,wave_frequency);
    auto forces_world = vessel_world_pose.Rot() * effects.force;
    ASSERT_TRUE(forces_world.X() < 0);
    ASSERT_TRUE(forces_world.Y() > 0);
    ASSERT_TRUE(effects.torque.X() > 0);
}

TEST_F(WaveTest, it_applies_positive_y_force_in_body_frame_when_the_apparent_wave_is_perpendicular_to_the_vessel)
{
    vessel_linear_vel = Vector3d({1, 0, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, 0}));
    wave_amplitude = Vector3d({1, 1, 0});
    wave_frequency = Vector3d({0.3, 0.3, 0});
    auto plugin = this->plugin();
    plugin.phase_x = M_PI/2;
    plugin.phase_y = M_PI/2;
    plugin.phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time,vessel_world_pose.Rot(), vessel_linear_vel, wave_amplitude,wave_frequency);
    auto forces_world = vessel_world_pose.Rot() * effects.force;
    ASSERT_NEAR(effects.force.X(), 0, 0.001);
    ASSERT_TRUE(effects.force.Y() > 0);
    ASSERT_NEAR(effects.torque.X(), 0, 0.001);
}

TEST_F(WaveTest, it_applies_negative_x_force_in_body_frame_when_the_apparent_wave_is_180_degrees_to_the_vessel)
{
    vessel_linear_vel = Vector3d({1, 0, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, 0}));
    wave_amplitude = Vector3d({-1, 0, 0});
    wave_frequency = Vector3d({0.3, 0.3, 0});
    auto plugin = this->plugin();
    plugin.phase_x = M_PI/2;
    plugin.phase_y = M_PI/2;
    plugin.phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time,vessel_world_pose.Rot(), vessel_linear_vel, wave_amplitude,wave_frequency);
    auto forces_world = vessel_world_pose.Rot() * effects.force;    
    ASSERT_TRUE(effects.force.X() < 0);
    ASSERT_NEAR(effects.force.Y(), 0, 0.001);
    ASSERT_NEAR(effects.torque.X(), 0, 0.001);
}

TEST_F(WaveTest, it_applies_positive_y_force_negative_x_force_and_positive_torque_in_body_frame_when_the_apparent_wave_is_negative_45_degrees_to_the_vessel)
{
    vessel_linear_vel = Vector3d({1, 0, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, 0}));
    wave_amplitude = Vector3d({-1, 2, 0});
    wave_frequency = Vector3d({0.3, 0.3, 0});
    auto plugin = this->plugin();
    plugin.phase_x = M_PI/2;
    plugin.phase_y = M_PI/2;
    plugin.phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time,vessel_world_pose.Rot(), vessel_linear_vel, wave_amplitude,wave_frequency);
    auto forces_world = vessel_world_pose.Rot() * effects.force;    
    ASSERT_TRUE(effects.force.X() < 0);
    ASSERT_TRUE(effects.force.Y() > 0);
    ASSERT_TRUE(effects.torque.X() > 0);
}

TEST_F(WaveTest, it_applies_neither_force_nor_torque_when_the_apparent_wave_is_zero)
{
    vessel_linear_vel = Vector3d({1, 1, 0});
    vessel_world_pose.Rot() = Quaterniond(Vector3d({0, 0, IGN_PI/4}));
    wave_amplitude = Vector3d({1, 1, 0});
    wave_frequency = Vector3d({0.3, 0.3, 0});
    auto plugin = this->plugin();
    plugin.phase_x = M_PI/2;
    plugin.phase_y = M_PI/2;
    plugin.phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time,vessel_world_pose.Rot(), vessel_linear_vel, wave_amplitude,wave_frequency);
    ASSERT_NEAR(effects.force.X(), 0, 0.001);
    ASSERT_NEAR(effects.force.Y(), 0, 0.001);
    ASSERT_NEAR(effects.torque.X(), 0, 0.001);
}