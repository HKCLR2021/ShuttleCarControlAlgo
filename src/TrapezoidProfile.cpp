#include "TrapezoidProfile.h"

#include <cmath>
#include <algorithm>
#include <spdlog/spdlog.h>

namespace frc {
    TrapezoidProfile::TrapezoidProfile(Constraints constraints)
            : m_constraints(constraints) {}

    TrapezoidProfile::State
    TrapezoidProfile::Calculate(Duration dt, State goal, State current) {
        m_direction = ShouldFlipAcceleration(current, goal) ? -1 : 1;
        m_current = Direct(current);
        m_timeFromStart += dt;
        goal = Direct(goal);
        if (m_current.velocity > m_constraints.maxVelocity) {
            m_current.velocity = m_constraints.maxVelocity;
        }

        // Deal with a possibly truncated motion profile (with nonzero initial or
        // final velocity) by calculating the parameters as if the profile began and
        // ended at zero velocity
        Duration cutoffBegin =
                m_current.velocity / m_constraints.maxAcceleration;
        Distance cutoffDistBegin =
                cutoffBegin * cutoffBegin * m_constraints.maxAcceleration / 2.0;

        Duration cutoffEnd = goal.velocity / m_constraints.maxAcceleration;
        Distance cutoffDistEnd =
                cutoffEnd * cutoffEnd * m_constraints.maxAcceleration / 2.0;

        // Now we can calculate the parameters as if it was a full trapezoid instead
        // of a truncated one

        Distance fullTrapezoidDist =
                cutoffDistBegin + (goal.position - m_current.position) + cutoffDistEnd;

        Duration accelerationTime =
                m_constraints.maxVelocity / m_constraints.maxAcceleration;

        Distance fullSpeedDist =
                fullTrapezoidDist - accelerationTime * accelerationTime * m_constraints.maxAcceleration;

        // SPDLOG_DEBUG("cutoffDistBegin {} | cutoffDistEnd {} | fullTrapezoidDist {} | fullSpeedDist {}",
        //           cutoffDistBegin, cutoffDistEnd, fullTrapezoidDist, fullSpeedDist);

        // Handle the case where the profile never reaches full speed
        if (fullSpeedDist < Distance{0}) {
            accelerationTime =
                    std::sqrt(fullTrapezoidDist / m_constraints.maxAcceleration);
            fullSpeedDist = Distance{0};
        }

        m_endAccel = accelerationTime - cutoffBegin;
        m_endFullSpeed = m_endAccel + fullSpeedDist / m_constraints.maxVelocity;
        m_endDeccel = m_endFullSpeed + accelerationTime - cutoffEnd;
        State result = m_current;
        // SPDLOG_DEBUG("current {} {}", result.position , result.velocity );
        SPDLOG_DEBUG("m_timeFromStart {:0.2f} >>> m_endAccel {:0.2f} | m_endFullSpeed {:0.2f} | m_endDeccel {:0.2f}",
                      m_timeFromStart, m_endAccel, m_endFullSpeed, m_endDeccel);


        if (dt < m_endAccel) { // acceleration stage
            result.velocity += dt * m_constraints.maxAcceleration;
            result.position +=
                    (m_current.velocity + result.velocity) / 2.0 * dt; // modified
            SPDLOG_DEBUG("<acc stage> pos inc [{:0.2f}], vel inc[{:0.2f}mm/s]",
                          (m_current.velocity + result.velocity) / 2.0 * dt, dt * m_constraints.maxAcceleration);

        } else if (dt < m_endFullSpeed) { // fullspeed stage
            result.velocity = m_constraints.maxVelocity;
            result.position += (m_current.velocity +
                                m_endAccel * m_constraints.maxAcceleration / 2.0) *
                               m_endAccel +
                               m_constraints.maxVelocity * (dt - m_endAccel);
            SPDLOG_DEBUG("<fullspeed stage> pos inc [{:0.2f}]", (m_current.velocity +
                                                                  m_endAccel * m_constraints.maxAcceleration / 2.0) *
                                                                 m_endAccel +
                                                                 m_constraints.maxVelocity * (dt - m_endAccel));

        } else if (dt <= m_endDeccel) { //deceleration stage
            result.velocity =
                    goal.velocity + (m_endDeccel - dt) * m_constraints.maxAcceleration;
            Duration timeLeft = m_endDeccel - dt;
            result.position =
                    goal.position -
                    (goal.velocity + timeLeft * m_constraints.maxAcceleration / 2.0) *
                    timeLeft;
            SPDLOG_DEBUG("dec stage");

        } else {
            result = goal;
        }
        // spdlog::warn("result {} {}", result.position , result.velocity );
        SPDLOG_INFO("<TrapezoidProfile> output: dt [{:0.2f}] pos[{:0.2f}/{:0.2f}], vel[{:0.2f}/{:0.2f} mm/s]",
                     dt, current.position, goal.position, current.velocity, goal.velocity);
        return Direct(result);
    }

    Duration TrapezoidProfile::TimeLeftUntil(Distance target) const {
        Distance position = m_current.position * m_direction;
        Velocity velocity = m_current.velocity * m_direction;

        Duration endAccel = m_endAccel * m_direction;
        Duration endFullSpeed = m_endFullSpeed * m_direction - endAccel;

        if (target < position) {
            endAccel *= -1.0;
            endFullSpeed *= -1.0;
            velocity *= -1.0;
        }

        endAccel = std::max(endAccel, 0.0);
        endFullSpeed = std::max(endFullSpeed, 0.0);

        const Acceleration acceleration = m_constraints.maxAcceleration;
        const Acceleration decceleration = -m_constraints.maxAcceleration;

        Distance distToTarget = std::abs(target - position);

        // if (distToTarget < Distance_t{1e-6}) {
        if (distToTarget < Distance{0}) {
            return 0;
        }

        Distance accelDist =
                velocity * endAccel + 0.5 * acceleration * endAccel * endAccel;

        Velocity deccelVelocity;
        if (endAccel > 0) {
            deccelVelocity = std::sqrt(
                    std::abs(velocity * velocity + 2 * acceleration * accelDist));
        } else {
            deccelVelocity = velocity;
        }

        Distance fullSpeedDist = m_constraints.maxVelocity * endFullSpeed;
        Distance deccelDist;

        if (accelDist > distToTarget) {
            accelDist = distToTarget;
            fullSpeedDist = Distance{0};
            deccelDist = Distance{0};
        } else if (accelDist + fullSpeedDist > distToTarget) {
            fullSpeedDist = distToTarget - accelDist;
            deccelDist = Distance{0};
        } else {
            deccelDist = distToTarget - fullSpeedDist - accelDist;
        }

        Duration accelTime =
                (-velocity + std::sqrt(std::abs(
                        velocity * velocity + 2 * acceleration * accelDist))) /
                acceleration;

        Duration deccelTime =
                (-deccelVelocity +
                 std::sqrt(std::abs(deccelVelocity * deccelVelocity +
                                    2 * decceleration * deccelDist))) /
                decceleration;

        Duration fullSpeedTime = fullSpeedDist / m_constraints.maxVelocity;

        return accelTime + fullSpeedTime + deccelTime;
    }
}  // namespace frc

