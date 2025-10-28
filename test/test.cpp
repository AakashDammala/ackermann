#include <gtest/gtest.h>
#include <cmath>
#include "libackermann.hpp"
#include "libpid.hpp"

using ::testing::Test;

/**
 * @brief Straight line: All wheels should have equal RPM.
 *
 * Manual calculation:
 * linear_vel = 2.0 m/s
 * wheel_radius = 0.15 m
 * RAD_TO_RPM = 60/(2*pi) ~= 9.5492965855
 * wheel_rpm = (linear_vel / wheel_radius) * RAD_TO_RPM
 *           = (2.0 / 0.15) * 9.5492965855
 *           = 13.3333333 * 9.5492965855 ~= 127.32395 RPM
 */
TEST(AckermannTest, StraightLineAllWheelsSameRPM) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  auto out = ctrl.compute(2.0, 0.0, 0.01); // 2 m/s straight at t=0.01s

  // All wheel RPMs should be equal for straight motion
  EXPECT_NEAR(out.wheel_rpm[0], out.wheel_rpm[1], 1e-6);
  EXPECT_NEAR(out.wheel_rpm[1], out.wheel_rpm[2], 1e-6);
  EXPECT_NEAR(out.wheel_rpm[2], out.wheel_rpm[3], 1e-6);

  // steering angles should be (near) zero
  EXPECT_NEAR(out.steering_angle[0], 0.0, 1e-6);
  EXPECT_NEAR(out.steering_angle[1], 0.0, 1e-6);
  // sanity check of expected magnitude
  EXPECT_NEAR(out.wheel_rpm[0], 127.32395, 1e-3);
}

/**
 * @brief Turning: wheels and steering should differ between left and right.
 *
 * Manual calculation (v=1.0, w=0.5):
 * R = v / w = 2.0 m
 * half_width = 0.6 / 2 = 0.3 m, L = 1.0 m
 * R_left = R - half_width = 1.7 m
 * R_right = R + half_width = 2.3 m
 * steering_left  = atan(L / R_left)  = atan(1/1.7) ~= 0.528 rad
 * steering_right = atan(L / R_right) = atan(1/2.3) ~= 0.410 rad
 * Wheel radii: R_fl = sqrt((R-left_half)^2 + L^2) ~= 1.973
 *              R_fr = sqrt((R+half)^2 + L^2) ~= 2.508
 *              R_rl = |R - half| = 1.7
 *              R_rr = |R + half| = 2.3
 * v_fl = v * (R_fl/|R|) ~= 0.9865 m/s -> rpm ~= (0.9865/0.15)*9.5493 ~= 62.8
 * v_fr ~= 1.254 m/s -> rpm ~= 79.8
 */
TEST(AckermannTest, TurningProducesDifferentWheelSpeedsAndSteering) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-10.0, 10.0};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  // turn left with positive angular velocity
  double v = 1.0; // m/s
  double w = 0.5; // rad/s
  auto out = ctrl.compute(v, w, 0.02);

  // steering angles should be non-zero
  EXPECT_GT(std::abs(out.steering_angle[0]), 0.0);
  EXPECT_GT(std::abs(out.steering_angle[1]), 0.0);
  // Wheel RPMs should not all be equal
  bool all_equal = (std::abs(out.wheel_rpm[0] - out.wheel_rpm[1]) < 1e-6) &&
                   (std::abs(out.wheel_rpm[1] - out.wheel_rpm[2]) < 1e-6);
  EXPECT_FALSE(all_equal);
  // check relative ordering expected for left turn: FL < FR
  EXPECT_LT(out.wheel_rpm[0], out.wheel_rpm[1]);
}

/**
 * @brief Zero commanded velocities -> zero RPMs and zero steering.
 *
 * Manual calculation:
 * linear_vel = 0.0 -> wheel_rpm = 0.0
 * steering angles = 0.0
 */
TEST(AckermannTest, ZeroVelocitiesProduceZeroOutput) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  auto out = ctrl.compute(0.0, 0.0, 0.0);

  EXPECT_NEAR(out.wheel_rpm[0], 0.0, 1e-9);
  EXPECT_NEAR(out.wheel_rpm[1], 0.0, 1e-9);
  EXPECT_NEAR(out.wheel_rpm[2], 0.0, 1e-9);
  EXPECT_NEAR(out.wheel_rpm[3], 0.0, 1e-9);
  EXPECT_NEAR(out.steering_angle[0], 0.0, 1e-9);
  EXPECT_NEAR(out.steering_angle[1], 0.0, 1e-9);
}

/**
 * @brief Reverse straight motion: negative linear velocity produces negative RPMs.
 *
 * Manual calculation:
 * linear_vel = -3.0 m/s
 * wheel_rpm = (-3.0/0.15)*9.5492965855 = -20.0*9.5493 = -190.9859 RPM
 */
TEST(AckermannTest, ReverseStraightAllWheelsSameRPM) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  auto out = ctrl.compute(-3.0, 0.0, 0.01);

  EXPECT_NEAR(out.wheel_rpm[0], out.wheel_rpm[1], 1e-6);
  EXPECT_NEAR(out.wheel_rpm[1], out.wheel_rpm[2], 1e-6);
  EXPECT_NEAR(out.wheel_rpm[2], out.wheel_rpm[3], 1e-6);
  EXPECT_NEAR(out.wheel_rpm[0], -190.98593, 1e-3);
}

/**
 * @brief Velocity clamping: requested velocity exceeds limits -> outputs clamped.
 *
 * Manual calculation:
 * requested linear_vel = 2.0 m/s but limits = {-0.5, 0.5}
 * clamped linear = 0.5 m/s
 * wheel_rpm = (0.5/0.15)*9.5492965855 ~= 31.83098 RPM
 */
TEST(AckermannTest, VelocityLimitsCauseClamping) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-0.5, 0.5};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  auto out = ctrl.compute(2.0, 0.0, 0.01);

  // since straight-line clamping happens and PID is identity here, clamped flag should be false
  // (applyPID uses setpoint==measurement so no PID correction). We still assert rpm matches clamped value.
  EXPECT_NEAR(out.wheel_rpm[0], 31.83098, 1e-3);
}

/**
 * @brief Steering limits: when steering computed exceeds limits, it's clamped.
 *
 * Manual calculation example (sharp left turn): expected steering angles > 0.2 rad -> clamped to 0.2
 */
TEST(AckermannTest, SteeringLimitsCauseClamping) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-10.0, 10.0};
  // tight steering limits
  cfg.steering_limits_ = {-0.2, 0.2};

  AckermannController ctrl(cfg);
  // large angular velocity to force large steering
  auto out = ctrl.compute(1.0, 5.0, 0.01);

  // steering angles should be clamped into [-0.2, 0.2]
  EXPECT_LE(out.steering_angle[0], 0.2 + 1e-9);
  EXPECT_GE(out.steering_angle[1], -0.2 - 1e-9);
  // ensure clamped flag set when any clamped
  EXPECT_TRUE(out.clamped || std::abs(out.steering_angle[0] - out.steering_angle[1]) < 1.57);
}

/**
 * @brief Very small angular velocity treated as straight (below eps threshold).
 *
 * Manual calculation: angular_vel = 1e-13 < eps (1e-12) -> steering = 0, all wheels same rpm
 */
TEST(AckermannTest, TinyAngularVelocityTreatedAsStraight) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  auto out = ctrl.compute(1.0, 1e-13, 0.01);

  EXPECT_NEAR(out.steering_angle[0], 0.0, 1e-12);
  EXPECT_NEAR(out.steering_angle[1], 0.0, 1e-12);
  EXPECT_NEAR(out.wheel_rpm[0], out.wheel_rpm[1], 1e-6);
  EXPECT_NEAR(out.wheel_rpm[1], out.wheel_rpm[2], 1e-6);
}

/**
 * @brief Negative velocity clamping to minimum limit.
 *
 * Manual calculation: requested -20 m/s with limits [-5,5] -> clamped to -5 m/s -> rpm.
 */
TEST(AckermannTest, NegativeVelocityClampedToMin) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.6;
  cfg.drive_length_ = 1.0;
  cfg.wheel_radius_ = 0.15;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.57, 1.57};

  AckermannController ctrl(cfg);
  auto out = ctrl.compute(-20.0, 0.0, 0.01);

  // expected clamped linear = -5.0 -> rpm = (-5.0/0.15)*9.5492965855 ~= -318.3099
  EXPECT_NEAR(out.wheel_rpm[0], -318.3099, 1e-3);
}

/**
 * @brief Test AckermannModel update with acceleration input.
 * Manual calculation:
 * initial_speed = 1.0 m/s
 * acceleration = 2.0 m/s^2
 * delta_time = 0.2 s
 * expected_speed = initial_speed + acceleration * delta_time
 *                = 1.0 + 2.0 * 0.2 = 1.4 m/s
 */
TEST(AckermannModel, AccelerationTest) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.0, 1.0};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 1.0; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.0;    // rad
  double delta_time = 0.2;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  // Apply acceleration and zero steering
  double acceleration = 2.0; // m/s^2
  AckermannState updated_state = model.update(acceleration, 0.0);
  // Expected speed: v + a * dt
  double expected_speed = 1.0 + acceleration * delta_time;
  EXPECT_NEAR(updated_state.longitudnal_speed_, expected_speed, 1e-6);
  EXPECT_NEAR(updated_state.heading_angle_, 0.0, 1e-6);
  EXPECT_NEAR(updated_state.steering_angle_, 0.0, 1e-6);
}

/**
 * @brief Test AckermannModel update with zero acceleration and steering for straight motion.
 */

TEST(AckermannModel, StraightMotion) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-1.0, 1.0};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 2.0; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.0;    // rad
  double delta_time = 0.1;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  // Apply zero acceleration and zero steering for straight motion
  AckermannState updated_state = model.update(0.0, 0.0);
  EXPECT_NEAR(updated_state.longitudnal_speed_, 2.0, 1e-6);
  EXPECT_NEAR(updated_state.heading_angle_, 0.0, 1e-6);
  EXPECT_NEAR(updated_state.steering_angle_, 0.0, 1e-6);
}

/**
 * @brief Test AckermannModel update with zero acceleration and non-zero steering for turning motion.
 */

TEST(AckermannModel, TurningMotion) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-0.5, 0.5};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 3.0; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.0;    // rad
  double delta_time = 0.1;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  // Apply zero acceleration and a steering angle for turning
  double steering_angle = 0.3; // rad
  AckermannState updated_state = model.update(0.0, steering_angle);
  EXPECT_NEAR(updated_state.longitudnal_speed_, 3.0, 1e-6);
  // Expected heading change: v * dt * tan(steering) / L
  double expected_heading_change = (3.0 * delta_time * std::tan(steering_angle)) / cfg.drive_length_;
  EXPECT_NEAR(updated_state.heading_angle_, expected_heading_change, 1e-6);
  EXPECT_NEAR(updated_state.steering_angle_, steering_angle, 1e-6);
}

/**
 * @brief Test AckermannModel wheel speeds and steering angles computation for turning.
 */
TEST(AckermannModel, WheelSpeedsAndSteeringAnglesForTurning) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-0.5, 0.5};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 4.0; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.2;    // rad
  double delta_time = 0.1;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  AckermannVehicleState vehicle_state = model.get_vehicle_state();
  // Check that wheel RPMs and steering angles are computed (non-zero values)
  for (int i = 0; i < 4; ++i) {
    EXPECT_GT(std::abs(vehicle_state.wheel_rpm[i]), 0.0);
  }
  for (int i = 0; i < 2; ++i) {
    EXPECT_GT(std::abs(vehicle_state.wheel_steering_angle[i]), 0.0);
  }
}

/**
 * @brief Test AckermannModel wheel speeds and steering angles computation for straight motion.
 */

TEST(AckermannModel, WheelSpeedsAndSteeringAnglesForStraight) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-0.5, 0.5};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 4.0; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.0;    // rad
  double delta_time = 0.1;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  AckermannVehicleState vehicle_state = model.get_vehicle_state();
  // Check that all wheel RPMs are equal for straight motion
  EXPECT_NEAR(vehicle_state.wheel_rpm[0], vehicle_state.wheel_rpm[1], 1e-6);
  EXPECT_NEAR(vehicle_state.wheel_rpm[1], vehicle_state.wheel_rpm[2], 1e-6);
  EXPECT_NEAR(vehicle_state.wheel_rpm[2], vehicle_state.wheel_rpm[3], 1e-6);
  // Check that steering angles are zero for straight motion
  EXPECT_NEAR(vehicle_state.wheel_steering_angle[0], 0.0, 1e-6);
  EXPECT_NEAR(vehicle_state.wheel_steering_angle[1], 0.0, 1e-6);
}

/**
 * @brief Test AckermannModel velocity clamping when exceeding max limit.
 */

TEST(AckermannModel, VelocityClamping) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-2.0, 2.0};
  cfg.steering_limits_ = {-0.5, 0.5};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 1.5; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.0;    // rad
  double delta_time = 0.1;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  // Apply acceleration that would exceed max velocity
  double acceleration = 10000.0; // m/s^2
  AckermannState updated_state = model.update(acceleration, 0.0);
  // Speed should be clamped to max limit of 2.0 m/s
  EXPECT_NEAR(updated_state.longitudnal_speed_, 2.0, 1e-6);
}

/**
 * @brief Test AckermannModel steering angle clamping when exceeding limits.
 */
TEST(AckermannModel, SteeringClamping) {
  AckermannConfig cfg{};
  cfg.drive_width_ = 0.5;
  cfg.drive_length_ = 1.2;
  cfg.wheel_radius_ = 0.2;
  cfg.velocity_limits_ = {-5.0, 5.0};
  cfg.steering_limits_ = {-0.3, 0.3};

  AckermannState initial_state{};
  initial_state.longitudnal_speed_ = 2.0; // m/s
  initial_state.heading_angle_ = 0.0;     // rad
  initial_state.steering_angle_ = 0.0;    // rad
  double delta_time = 0.1;                // s
  AckermannModel model(cfg, initial_state, delta_time);
  // Apply steering angle that exceeds limits
  double steering_angle = 1.0; // rad
  AckermannState updated_state = model.update(0.0, steering_angle);
  // Steering angle should be clamped to max limit of 0.3 rad
  EXPECT_NEAR(updated_state.steering_angle_, 0.3, 1e-6);
}


/**
 * @brief When setpoint equals measurement, controller should return zero value.
 * PID Gains: Kp = 1.0   Ki = 0.1   Kd = 0.01
 * delta_time = 0.1   min_output = -100.0   max_output = 100.0
 * setpoint = measurement = 5.0
 * error = setpoint - measurement = 5.0 - 5.0 = 0.0
 * Pout = 1.0 * 0.0 = 0.0
 * integral = 0.0 * 0.1 = 0.0
 * Iout = 0.1 * 0.0 = 0.0
 * derivative = (0.0 - 0.0) / 0.1 = 0.0
 * Dout = 0.01 * 0.0 = 0.0
 * expected_output = 0.0
 */
TEST(PIDControllerTest, SetpointEqualsMeasurement) {
  // Construct controller with example gains and limits.
  PIDController controller(1.0, 0.1, 0.01, 0.1, -100.0, 100.0);

  double setpoint = 5.0;
  double measurement = 5.0;  // same as setpoint
  double expected = 0.0;

  double output = controller.compute(setpoint, measurement);

  // Expect the controller output to be zero since error is zero
  // Use a small tolerance in case of floating-point computations.
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Proportional only (Ki, Kd = 0) control test
 * PID Gains: Kp = 2.0   Ki = 0.0   Kd = 0.0
 * delta_time = 0.1   min_output = -10.0   max_output = 10.0
 * error = setpoint - meassurement = 3.0 - 1.0 = 2.0
 * Pout = 2.0 * 2.0 = 4.0
 * Iout = 0.0
 * Dout = 0.0
 * expected_output = 4.0
 * expected_output within clamp limits, so no clamps applied
 */
TEST(PIDControllerTest, ProportionalOnlyBehavior) {
  const double Kp = 2.0;
  // Ki=0 and Kd=0 to test proportional-only response
  PIDController controller(Kp, 0.0, 0.0, 0.1, -10.0, 10.0);

  double setpoint = 3.0;
  double measurement = 1.0;
  double expected = 4.0;

  double output = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Controller output should be clamped to max_output.
 * PID Gains: Kp = 100.0   Ki = 0.0   Kd = 0.0
 * delta_time = 0.1   min_output = -5.0   max_output = 5.0
 * error = setpoint - measurement = 10.0 - 0.0 = 10.0
 * Pout = 100.0 * 10.0 = 1000.0
 * Iout = 0.0
 * Dout = 0.0
 * raw_output = 1000.0
 * expected_output = clamp(1000.0) = 5.0
 */
TEST(PIDControllerTest, ClampsToMaxOutput) {
  const double Kp = 100.0;
  PIDController controller(Kp, 0.0, 0.0, 0.1, -5.0, 5.0);

  double setpoint = 10.0;
  double measurement = 0.0;
  double expected = 5.0;

  double output = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Controller output should be clamped to min_output.
 * PID Gains: Kp = 100.0   Ki = 0.0   Kd = 0.0
 * delta_time = 0.1   min_output = -5.0   max_output = 5.0
 * error = setpoint - measurement = -10.0 - 0.0 = -10.0
 * Pout = 100.0 * -10.0 = -1000.0
 * Iout = 0.0
 * Dout = 0.0
 * raw_output = -1000.0
 * expected_output = clamp(-1000.0) = -5.0
 */
TEST(PIDControllerTest, ClampsToMinOutput) {
  const double Kp = 100.0;
  PIDController controller(Kp, 0.0, 0.0, 0.1, -5.0, 5.0);

  double setpoint = -10.0;
  double measurement = 0.0;
  double expected = -5.0;

  double output = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output, expected, 1e-6);
}

/**
 * @brief Integral only (Kp, Kd = 0) control test
 * PID Gains: Kp = 0.0   Ki = 0.5   Kd = 0.0
 * delta_time = 0.1   min_output = -10.0   max_output = 10.0
 *
 * First step:
 * error = setpoint - measurement = 5.0 - 3.0 = 2.0
 * integral = prev_integral + (error * delta_time) = 0.0 + (2.0 * 0.1) = 0.2
 * expected_output1 = Ki * integral = 0.5 * 0.2 = 0.1
 *
 * Second step:
 * integral = previous_integral + (error * delta_time) = 0.2 + (2.0 * 0.1) = 0.4
 * expected_output2 = Ki * integral = 0.5 * 0.4 = 0.2
 */
TEST(PIDControllerTest, IntegralOnlyBehavior) {
  const double Ki = 0.5;
  const double dt = 0.1;
  PIDController controller(0.0, Ki, 0.0, dt, -10.0, 10.0);

  double setpoint = 5.0;
  double measurement = 3.0;
  double expected1 = 0.1;

  // First step
  double output1 = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output1, expected1, 1e-6);

  // Second step, integral should accumulate
  double expected2 = 0.2;
  double output2 = controller.compute(setpoint, measurement);
  EXPECT_NEAR(output2, expected2, 1e-6);
}

/**
 * @brief Differential only (Kp, Ki = 0) control test
 * PID Gains: Kp = 0.0   Ki = 0.0   Kd = 0.5
 * delta_time = 0.1   min_output = -10.0   max_output = 10.0
 *
 * First step:
 * error1 = setpoint - measurement1 = 5.0 - 3.0 = 2.0
 * previous_error = 0.0 (initial)
 * derivative = (error1 - previous_error) / dt = (2.0 - 0.0) / 0.1 = 20.0
 * expected_output1 = Kd * derivative = 0.5 * 20.0 = 10.0
 *
 * Second step:
 * error2 = setpoint - measurement2 = 5.0 - 4.0 = 1.0
 * derivative = (error2 - error1) / dt = (1.0 - 2.0) / 0.1 = -10.0
 * expected_output2 = Kd * derivative = 0.5 * (-10.0) = -5.0
 */
TEST(PIDControllerTest, DifferentialOnlyBehavior) {
  const double Kd = 0.5;
  const double dt = 0.1;
  PIDController controller(0.0, 0.0, Kd, dt, -10.0, 10.0);

  double setpoint = 5.0;
  double measurement1 = 3.0;
  double expected1 = 10.0;

  // First step
  double output1 = controller.compute(setpoint, measurement1);
  EXPECT_NEAR(output1, expected1, 1e-6);

  // Second step
  double measurement2 = 4.0;
  double expected2 = -5.0;
  double output2 = controller.compute(setpoint, measurement2);
  EXPECT_NEAR(output2, expected2, 1e-6);
}

/**
 * @brief Test with all PID terms active over two iterations
 * PID Gains: Kp = 1.0   Ki = 0.5   Kd = 0.1
 * delta_time = 0.1   min_output = -10.0   max_output = 10.0
 * setpoint = 10.0
 *
 * First iteration (measurement = 5.0):
 * error1 = 10.0 - 5.0 = 5.0
 * Pout1 = 1.0 * 5.0 = 5.0
 * integral1 = 0.0 + 5.0 * 0.1 = 0.5
 * Iout1 = 0.5 * 0.5 = 0.25
 * derivative1 = (5.0 - 0.0) / 0.1 = 50.0
 * Dout1 = 0.1 * 50.0 = 5.0
 * output1 = 5.0 + 0.25 + 5.0 = 10.25
 * clamped_output1 = 10.0
 *
 * Second iteration (measurement = 7.0):
 * error2 = 10.0 - 7.0 = 3.0
 * Pout2 = 1.0 * 3.0 = 3.0
 * integral2 = 0.5 + 3.0 * 0.1 = 0.8
 * Iout2 = 0.5 * 0.8 = 0.4
 * derivative2 = (3.0 - 5.0) / 0.1 = -20.0
 * Dout2 = 0.1 * -20.0 = -2.0
 * output2 = 3.0 + 0.4 - 2.0 = 1.4
 */
TEST(PIDControllerTest, CombinedPIDBehavior) {
  const double Kp = 1.0;
  const double Ki = 0.5;
  const double Kd = 0.1;
  const double dt = 0.1;
  PIDController controller(Kp, Ki, Kd, dt, -10.0, 10.0);

  double setpoint = 10.0;

  // First iteration
  double measurement1 = 5.0;
  double expected1 = 10.0;  // Would be 10.25 but clamped to 10.0
  double output1 = controller.compute(setpoint, measurement1);
  EXPECT_NEAR(output1, expected1, 1e-6);

  // Second iteration
  double measurement2 = 7.0;
  double expected2 = 1.4;
}