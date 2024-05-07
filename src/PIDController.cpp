#include "PIDController.h"

#include <cmath>
#include <algorithm>

#include <spdlog/spdlog.h>

namespace frc {
    PIDController::PIDController(double Kp, double Ki, double Kd, double period)
            : m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_period(period) {
        if (period <= 0) {
            m_period = 0.03;
        }
    }

    void PIDController::SetPID(double Kp, double Ki, double Kd) {
        m_Kp = Kp;
        m_Ki = Ki;
        m_Kd = Kd;
    }

    void PIDController::SetP(double Kp) {
        m_Kp = Kp;
    }

    void PIDController::SetI(double Ki) {
        m_Ki = Ki;
    }

    void PIDController::SetD(double Kd) {
        m_Kd = Kd;
    }

    void PIDController::SetIZone(double iZone) {
        if (iZone < 0) {
            // wpi::math::MathSharedStore::ReportError(
            //     "IZone must be a non-negative number, got {}!", iZone);
            // report error
            return;
        }
        m_iZone = iZone;
    }

    double PIDController::GetP() const {
        return m_Kp;
    }

    double PIDController::GetI() const {
        return m_Ki;
    }

    double PIDController::GetD() const {
        return m_Kd;
    }

    double PIDController::GetIZone() const {
        return m_iZone;
    }

    double PIDController::GetPeriod() const {
        return m_period;
    }

    double PIDController::GetPositionTolerance() const {
        return m_positionTolerance;
    }

    double PIDController::GetVelocityTolerance() const {
        return m_velocityTolerance;
    }

    void PIDController::SetSetpoint(double setpoint) {
        m_setpoint = setpoint;
        m_haveSetpoint = true;

        // descret time case
        m_positionError = m_setpoint - m_measurement;

        m_velocityError = (m_positionError - m_prevError) / m_period;
    }

    double PIDController::GetSetpoint() const {
        return m_setpoint;
    }

    bool PIDController::AtSetpoint() const {
        return m_haveMeasurement && m_haveSetpoint &&
               std::abs(m_positionError) < m_positionTolerance &&
               std::abs(m_velocityError) < m_velocityTolerance;
    }

    void PIDController::SetIntegratorRange(double minimumIntegral,
                                           double maximumIntegral) {
        m_minimumIntegral = minimumIntegral;
        m_maximumIntegral = maximumIntegral;
    }

    void PIDController::SetTolerance(double positionTolerance,
                                     double velocityTolerance) {
        m_positionTolerance = positionTolerance;
        m_velocityTolerance = velocityTolerance;
    }

    double PIDController::GetPositionError() const {
        return m_positionError;
    }

    double PIDController::GetVelocityError() const {
        return m_velocityError;
    }

    double PIDController::Calculate(double measurement) {
        m_measurement = measurement;
        m_prevError = m_positionError;
        m_haveMeasurement = true;

        // descret time case
        m_positionError = m_setpoint - m_measurement;

        m_velocityError = (m_positionError - m_prevError) / m_period;

        // If the absolute value of the position error is outside of IZone, reset the
        // total error
        if (std::abs(m_positionError) > m_iZone) {
            m_totalError = 0;
        } else if (m_Ki != 0) {
            // // std::clamp() requires C++17
            // m_totalError =
            //     std::clamp(m_totalError + m_positionError * m_period,
            //                m_minimumIntegral / m_Ki, m_maximumIntegral / m_Ki);
            SPDLOG_INFO(" m_totalError {:0.2f} + {:0.2f} * {:0.2f}",m_totalError, m_positionError, m_period);
            m_totalError = (m_totalError + m_positionError * m_period);

            // clamping to a reasonable range
            m_totalError = std::max(m_totalError, m_minimumIntegral);
            m_totalError = std::min(m_totalError, m_maximumIntegral);
        }
        SPDLOG_INFO(" dt {:0.3f}, pos[{:0.2f}/{:0.2f}], pTerm {:0.2f}, iTerm {:0.2f}, dTerm {:0.2f}",
                    GetPeriod(), measurement, m_setpoint,
                    m_Kp * m_positionError, m_Ki * m_totalError, m_Kd * m_velocityError);
        return m_Kp * m_positionError + m_Ki * m_totalError + m_Kd * m_velocityError;
    }

    double PIDController::Calculate(double measurement, double setpoint) {
        m_setpoint = setpoint;
        m_haveSetpoint = true;
        return Calculate(measurement);
    }

    double PIDController::Calculate(double measurement, double setpoint, double period) {
        m_setpoint = setpoint;
        m_haveSetpoint = true;
        m_period = period;
        return Calculate(measurement);
    }

    void PIDController::Reset() {
        m_positionError = 0;
        m_prevError = 0;
        m_totalError = 0;
        m_velocityError = 0;
        m_haveMeasurement = false;
    }
}  // namespace frc
