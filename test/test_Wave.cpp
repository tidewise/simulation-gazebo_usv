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

    WaveTest()
    {
        parameters.torque_constant = 0.4;
    }

    Wave plugin()
    {
        return Wave(parameters);
    }
    
};

TEST_F(WaveTest, it_applies_force_in_the_world_x_axis)
{
    wave_amplitude = Vector3d({1, 0, 0});
    wave_frequency = Vector3d({0.3, 0, 0});
    auto plugin = this->plugin();
    plugin.m_phase_x = M_PI/2;
    plugin.m_phase_y = M_PI/2;
    plugin.m_phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time, wave_amplitude,wave_frequency);
    ASSERT_TRUE(effects.force.X() > 0);
    ASSERT_NEAR(effects.force.Y(), 0, 0.001);
    ASSERT_NEAR(effects.force.Z(), 0, 0.001);
}

TEST_F(WaveTest, it_applies_force_in_the_world_x_axis_at_different_times)
{
    wave_amplitude = Vector3d({1, 0, 0});
    wave_frequency = Vector3d({0.3, 0, 0});
    auto plugin = this->plugin();
    plugin.m_phase_x = M_PI/2;
    plugin.m_phase_y = M_PI/2;
    plugin.m_phase_z = M_PI/2;
    double current_time = M_PI;

    auto effects = plugin.computeEffects(current_time, wave_amplitude,wave_frequency);
    ASSERT_TRUE(effects.force.X() > 0);
    ASSERT_NEAR(effects.force.Y(), 0, 0.001);
    ASSERT_NEAR(effects.force.Z(), 0, 0.001);
}

TEST_F(WaveTest, it_applies_force_in_the_world_y_axis)
{
    wave_amplitude = Vector3d({0, 1, 0});
    wave_frequency = Vector3d({0, 0.3, 0});
    auto plugin = this->plugin();
    plugin.m_phase_x = M_PI/2;
    plugin.m_phase_y = M_PI/2;
    plugin.m_phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time, wave_amplitude,wave_frequency);
    ASSERT_TRUE(effects.force.Y() > 0);
    ASSERT_NEAR(effects.force.X(), 0, 0.001);
    ASSERT_NEAR(effects.force.Z(), 0, 0.001);
}

TEST_F(WaveTest, it_applies_force_in_the_world_z_axis)
{
    wave_amplitude = Vector3d({0, 0, 1});
    wave_frequency = Vector3d({0, 0, 0.3});
    auto plugin = this->plugin();
    plugin.m_phase_x = M_PI/2;
    plugin.m_phase_y = M_PI/2;
    plugin.m_phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time, wave_amplitude,wave_frequency);
    ASSERT_TRUE(effects.force.Z() > 0);
    ASSERT_NEAR(effects.force.X(), 0, 0.001);
    ASSERT_NEAR(effects.force.Y(), 0, 0.001);
}

TEST_F(WaveTest, it_applies_negative_force_in_world_x_positive_force_in_y_and_no_torque_when_the_wave_in_distructive_interference_to_an_unactuated_vessel)
{
    wave_amplitude = Vector3d({1, 1, 0});
    wave_frequency = Vector3d({0.3, 0.3, 0});
    auto plugin = this->plugin();
    plugin.m_phase_x = M_PI/2;
    plugin.m_phase_y = -M_PI/2;
    plugin.m_phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time, wave_amplitude,wave_frequency);
    ASSERT_TRUE(effects.force.X() > 0);
    ASSERT_TRUE(effects.force.Y() < 0);
    ASSERT_NEAR(effects.torque.X(), 0, 0.001);
}

TEST_F(WaveTest, it_applies_neither_force_nor_torque_when_the_wave_is_zero)
{
    wave_amplitude = Vector3d({0, 0, 0});
    wave_frequency = Vector3d({0.3, 0.3, 0});
    auto plugin = this->plugin();
    plugin.m_phase_x = M_PI/2;
    plugin.m_phase_y = M_PI/2;
    plugin.m_phase_z = M_PI/2;
    double current_time = 0;

    auto effects = plugin.computeEffects(current_time, wave_amplitude,wave_frequency);
    ASSERT_NEAR(effects.force.X(), 0, 0.001);
    ASSERT_NEAR(effects.force.Y(), 0, 0.001);
    ASSERT_NEAR(effects.torque.X(), 0, 0.001);
}