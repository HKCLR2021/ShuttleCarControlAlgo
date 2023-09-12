#pragma once

#include "TrapezoidProfile.h"
#include "PIDController.h"

namespace frc {

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid
 * profile.
 */
class ProfiledPIDController {
 public:
  using Distance = double;
  using Velocity = double;
  using Acceleration = double;
  using State = typename TrapezoidProfile::State;
  using Constraints = typename TrapezoidProfile::Constraints;

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and
   * Kd. Users should call reset() when they first start running the controller
   * to avoid unwanted behavior.
   *
   * @param Kp          The proportional coefficient.
   * @param Ki          The integral coefficient.
   * @param Kd          The derivative coefficient.
   * @param constraints Velocity and acceleration constraints for goal.
   * @param period      The period between controller updates in seconds. The
   *                    default is 20 milliseconds.
   */
  ProfiledPIDController(double Kp, double Ki, double Kd,
                        Constraints constraints, Duration period = 0.02)
      : m_controller(Kp, Ki, Kd, period), m_constraints(constraints) { }

  ~ProfiledPIDController() = default;

  ProfiledPIDController(const ProfiledPIDController&) = default;
  ProfiledPIDController& operator=(const ProfiledPIDController&) = default;
  ProfiledPIDController(ProfiledPIDController&&) = default;
  ProfiledPIDController& operator=(ProfiledPIDController&&) = default;

  /**
   * Sets the PID Controller gain parameters.
   *
   * Sets the proportional, integral, and differential coefficients.
   *
   * @param Kp Proportional coefficient
   * @param Ki Integral coefficient
   * @param Kd Differential coefficient
   */
  void SetPID(double Kp, double Ki, double Kd) {
    m_controller.SetPID(Kp, Ki, Kd);
  }

  /**
   * Sets the proportional coefficient of the PID controller gain.
   *
   * @param Kp proportional coefficient
   */
  void SetP(double Kp) { m_controller.SetP(Kp); }

  /**
   * Sets the integral coefficient of the PID controller gain.
   *
   * @param Ki integral coefficient
   */
  void SetI(double Ki) { m_controller.SetI(Ki); }

  /**
   * Sets the differential coefficient of the PID controller gain.
   *
   * @param Kd differential coefficient
   */
  void SetD(double Kd) { m_controller.SetD(Kd); }

  /**
   * Sets the IZone range. When the absolute value of the position error is
   * greater than IZone, the total accumulated error will reset to zero,
   * disabling integral gain until the absolute value of the position error is
   * less than IZone. This is used to prevent integral windup. Must be
   * non-negative. Passing a value of zero will effectively disable integral
   * gain. Passing a value of infinity disables IZone functionality.
   *
   * @param iZone Maximum magnitude of error to allow integral control.
   */
  void SetIZone(double iZone) { m_controller.SetIZone(iZone); }

  /**
   * Gets the proportional coefficient.
   *
   * @return proportional coefficient
   */
  double GetP() const { return m_controller.GetP(); }

  /**
   * Gets the integral coefficient.
   *
   * @return integral coefficient
   */
  double GetI() const { return m_controller.GetI(); }

  /**
   * Gets the differential coefficient.
   *
   * @return differential coefficient
   */
  double GetD() const { return m_controller.GetD(); }

  /**
   * Get the IZone range.
   *
   * @return Maximum magnitude of error to allow integral control.
   */
  double GetIZone() const { return m_controller.GetIZone(); }

  /**
   * Gets the period of this controller.
   *
   * @return The period of the controller.
   */
  Duration GetPeriod() const { return m_controller.GetPeriod(); }

  /**
   * Gets the position tolerance of this controller.
   *
   * @return The position tolerance of the controller.
   */
  double GetPositionTolerance() const {
    return m_controller.GetPositionTolerance();
  }

  /**
   * Gets the velocity tolerance of this controller.
   *
   * @return The velocity tolerance of the controller.
   */
  double GetVelocityTolerance() const {
    return m_controller.GetVelocityTolerance();
  }

  /**
   * Sets the goal for the ProfiledPIDController.
   *
   * @param goal The desired unprofiled setpoint.
   */
  void SetGoal(State goal) { m_goal = goal; }

  /**
   * Sets the goal for the ProfiledPIDController.
   *
   * @param goal The desired unprofiled setpoint.
   */
  void SetGoal(Distance goal) { m_goal = {goal, Velocity{0}}; }

  /**
   * Gets the goal for the ProfiledPIDController.
   */
  State GetGoal() const { return m_goal; }

  /**
   * Returns true if the error is within the tolerance of the error.
   *
   * This will return false until at least one input value has been computed.
   */
  // bool AtGoal() const { return AtSetpoint() && m_goal == m_setpoint; }

  /**
   * Set velocity and acceleration constraints for goal.
   *
   * @param constraints Velocity and acceleration constraints for goal.
   */
  void SetConstraints(Constraints constraints) { m_constraints = constraints; }

  /**
   * Get the velocity and acceleration constraints for this controller.
   * @return Velocity and acceleration constraints.
   */
  Constraints GetConstraints() { return m_constraints; }

  /**
   * Returns the current setpoint of the ProfiledPIDController.
   *
   * @return The current setpoint.
   */
  State GetSetpoint() const { return m_setpoint; }

  /**
   * Returns true if the error is within the tolerance of the error.
   *
   * Currently this just reports on target as the actual value passes through
   * the setpoint. Ideally it should be based on being within the tolerance for
   * some period of time.
   *
   * This will return false until at least one input value has been computed.
   */
  bool AtSetpoint() const { return m_controller.AtSetpoint(); }


  /**
   * Sets the minimum and maximum values for the integrator.
   *
   * When the cap is reached, the integrator value is added to the controller
   * output rather than the integrator value times the integral gain.
   *
   * @param minimumIntegral The minimum value of the integrator.
   * @param maximumIntegral The maximum value of the integrator.
   */
  void SetIntegratorRange(double minimumIntegral, double maximumIntegral) {
    m_controller.SetIntegratorRange(minimumIntegral, maximumIntegral);
  }

  /**
   * Sets the error which is considered tolerable for use with
   * AtSetpoint().
   *
   * @param positionTolerance Position error which is tolerable.
   * @param velocityTolerance Velocity error which is tolerable.
   */
  void SetTolerance(Distance positionTolerance,
                    Velocity velocityTolerance = Velocity{
                        std::numeric_limits<double>::infinity()}) {
    m_controller.SetTolerance(positionTolerance,
                              velocityTolerance);
  }

  /**
   * Returns the difference between the setpoint and the measurement.
   *
   * @return The error.
   */
  Distance GetPositionError() const {
    return Distance{m_controller.GetPositionError()};
  }

  /**
   * Returns the change in error per second.
   */
  Velocity GetVelocityError() const {
    return Velocity{m_controller.GetVelocityError()};
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   */
  double Calculate(Distance measurement) {
    frc::TrapezoidProfile profile{m_constraints};
    m_setpoint = profile.Calculate(GetPeriod(), m_goal, m_setpoint);
    return m_controller.Calculate(measurement,
                                  m_setpoint.position);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal The new goal of the controller.
   */
  double Calculate(Distance measurement, State goal) {
    SetGoal(goal);
    return Calculate(measurement);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal The new goal of the controller.
   */
  double Calculate(Distance measurement, Distance goal) {
    SetGoal(goal);
    return Calculate(measurement);
  }

  /**
   * Returns the next output of the PID controller.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal        The new goal of the controller.
   * @param constraints Velocity and acceleration constraints for goal.
   */
  double Calculate(
      Distance measurement, Distance goal,
       frc::TrapezoidProfile::Constraints constraints) {
    SetConstraints(constraints);
    return Calculate(measurement, goal);
  }

  /**
   * Reset the previous error and the integral term.
   *
   * @param measurement The current measured State of the system.
   */
  void Reset(const State& measurement) {
    m_controller.Reset();
    m_setpoint = measurement;
  }

  /**
   * Reset the previous error and the integral term.
   *
   * @param measuredPosition The current measured position of the system.
   * @param measuredVelocity The current measured velocity of the system.
   */
  void Reset(Distance measuredPosition, Velocity measuredVelocity) {
    Reset(State{measuredPosition, measuredVelocity});
  }

  /**
   * Reset the previous error and the integral term.
   *
   * @param measuredPosition The current measured position of the system. The
   * velocity is assumed to be zero.
   */
  void Reset(Distance measuredPosition) {
    Reset(measuredPosition, Velocity{0});
  }

 private:
  PIDController m_controller;
  Distance m_minimumInput{0};
  Distance m_maximumInput{0};
  typename frc::TrapezoidProfile::State m_goal;
  typename frc::TrapezoidProfile::State m_setpoint;
  typename frc::TrapezoidProfile::Constraints m_constraints;
};

}  // namespace frc
