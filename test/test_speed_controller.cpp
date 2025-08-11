///////////////////////////////////////////////////////////////////////////////
// 
// Programmer: Victoria Salvati
// Date: August 10, 2025
// Filename: test_speed_controller.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "speed_controller.hpp"

////////////////////////////////////////////////////////////////////////////////

class TestSpeedController : public ::testing::Test 
{
protected:
    const bool silence_logging = true;

    void SetUp() override 
    {
        rclcpp::init(0, nullptr);
    }
    void TearDown() override 
    {
        rclcpp::shutdown();
    }
};

////////////////////////////////////////////////////////////////////////////////

TEST_F(TestSpeedController, TestDefaultConstructor_CorrectDefaults) 
{
    // arrange
    const bool expected_is_estop_on = false;
    const float expected_min_laser_dist_meters = 2.0;
    const SpeedState expected_speed_state = SpeedState::FULL_SPEED;

    // act
    auto sc = std::make_shared<SpeedController>(silence_logging);
 
    // assert
    ASSERT_EQ(sc->get_is_estop_on(), expected_is_estop_on);
    ASSERT_EQ(sc->get_min_laser_dist_meters(), expected_min_laser_dist_meters);
    ASSERT_EQ(sc->get_speed_state(), expected_speed_state);
}

TEST_F(TestSpeedController, TestDetermineSpeedState_EStopOn_FarSensorReading) 
{
    // arrange
    const bool estop = true;
    const float far_sensor_reading = 1.6;
    const SpeedState expected_speed_state = SpeedState::STOP;

    // act
    auto sc = std::make_shared<SpeedController>(silence_logging, estop, far_sensor_reading);
    
    // assert
    ASSERT_EQ(sc->get_speed_state(), expected_speed_state);
}

TEST_F(TestSpeedController, TestDetermineSpeedState_EStopOn_CloseSensorReading) 
{
    // arrange
    const bool estop = true;
    const float close_sensor_reading = 0.6;
    const SpeedState expected_speed_state = SpeedState::STOP;

    // act
    auto sc = std::make_shared<SpeedController>(silence_logging, estop, close_sensor_reading);
    
    // assert
    ASSERT_EQ(sc->get_speed_state(), expected_speed_state);
}

TEST_F(TestSpeedController, TestDetermineSpeedState_TooCloseSensorReading) 
{
    // arrange
    const bool estop = true;
    const float too_close_sensor_reading = 0.2;
    const SpeedState expected_speed_state = SpeedState::STOP;

    // act
    auto sc = std::make_shared<SpeedController>(silence_logging, estop, too_close_sensor_reading);
    
    // assert
    ASSERT_EQ(sc->get_speed_state(), expected_speed_state);
}

TEST_F(TestSpeedController, TestDetermineSpeedState_EStopOff_FarSensorReading) 
{
    // arrange
    const bool estop = false;
    const float far_sensor_reading = 1.6;
    const SpeedState expected_speed_state1 = SpeedState::FULL_SPEED;

    // act
    auto sc = std::make_shared<SpeedController>(silence_logging, estop, far_sensor_reading);
   
    // assert
    ASSERT_EQ(sc->get_speed_state(), expected_speed_state1);
}

TEST_F(TestSpeedController, TestDetermineSpeedState_EStopOff_CloseSensorReading) 
{
    // arrange
    const bool estop = false;
    const float close_sensor_reading = 0.6;
    const SpeedState expected_speed_state2 = SpeedState::SLOW;

    // act
    auto sc = std::make_shared<SpeedController>(silence_logging, estop, close_sensor_reading);
   
    // assert
    ASSERT_EQ(sc->get_speed_state(), expected_speed_state2);
}

TEST_F(TestSpeedController, TestDetermineSpeedState_EStopOff_TooCloseSensorReading) 
{
    // arrange
    const bool estop = false;
    const float too_close_sensor_reading = 0.2;
    const SpeedState expected_speed_state3 = SpeedState::STOP;

    // act
    auto sc = std::make_shared<SpeedController>(silence_logging, estop, too_close_sensor_reading);
   
    // assert
    ASSERT_EQ(sc->get_speed_state(), expected_speed_state3);
}

////////////////////////////////////////////////////////////////////////////////
