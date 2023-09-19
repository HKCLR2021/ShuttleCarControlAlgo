#pragma once

namespace frc {
    using Duration = double;

    /**
     * A trapezoid-shaped velocity profile.
     *
     * While this class can be used for a profiled movement from start to finish,
     * the intended usage is to filter a reference's dynamics based on trapezoidal
     * velocity constraints. To compute the reference obeying this constraint, do
     * the following.
     *
     * Initialization:
     * @code{.cpp}
     * TrapezoidProfile::Constraints constraints{kMaxV, kMaxA};
     * double previousProfiledReference = initialReference;
     * TrapezoidProfile profile{constraints};
     * @endcode
     *
     * Run on update:
     * @code{.cpp}
     * previousProfiledReference = profile.Calculate(timeSincePreviousUpdate,
     *                                               unprofiledReference,
     *                                               previousProfiledReference);
     * @endcode
     *
     * where `unprofiledReference` is free to change between calls. Note that when
     * the unprofiled reference is within the constraints, `Calculate()` returns the
     * unprofiled reference unchanged.
     *
     * Otherwise, a timer can be started to provide monotonic values for
     * `Calculate()` and to determine when the profile has completed via
     * `IsFinished()`.
     */
    class TrapezoidProfile {
    public:
        using Distance = double;
        using Velocity = double;
        using Acceleration = double;
        // using Duration = std::chrono::duration<double>;

        class Constraints {
        public:
            Constraints() = default;

            Constraints(Velocity maxVelocity_, Acceleration maxAcceleration_)
                    : maxVelocity{maxVelocity_}, maxAcceleration{maxAcceleration_} {}

            Velocity maxVelocity{0};
            Acceleration maxAcceleration{0};
        };

        class State {
        public:
            Distance position{0};
            Velocity velocity{0};
            // bool operator==(const State&) const = default;
        };

        /**
         * Construct a TrapezoidProfile.
         *
         * @param constraints The constraints on the profile, like maximum velocity.
         */
        TrapezoidProfile(Constraints constraints);  // NOLINT

        // /**
        //  * Construct a TrapezoidProfile.
        //  *
        //  * @param constraints The constraints on the profile, like maximum velocity.
        //  * @param goal        The desired state when the profile is complete.
        //  * @param initial     The initial state (usually the current state).
        //  * @deprecated Pass the desired and current state into calculate instead of
        //  * constructing a new TrapezoidProfile with the desired and current state
        //  */
        // TrapezoidProfile(Constraints constraints, State goal,
        //                  State initial = State{Distance_t{0}, Velocity_t{0}});

        TrapezoidProfile(const TrapezoidProfile &) = default;

        TrapezoidProfile &operator=(const TrapezoidProfile &) = default;

        TrapezoidProfile(TrapezoidProfile &&) = default;

        TrapezoidProfile &operator=(TrapezoidProfile &&) = default;

        /**
         * Calculate the correct position and velocity for the profile at a time t
         * where the beginning of the profile was at time t = 0.
         *
         * @param t The time since the beginning of the profile.
         * @param goal        The desired state when the profile is complete.
         * @param current     The initial state (usually the current state).
         */
        State Calculate(Duration t, State goal, State current);

        /**
         * Returns the time left until a target distance in the profile is reached.
         *
         * @param target The target distance.
         */
        Duration TimeLeftUntil(Distance target) const;

        /**
         * Returns the total time the profile takes to reach the goal.
         */
        Duration TotalTime() const { return m_timeFromStart; }

        /**
         * Returns true if the profile has reached the goal.
         *
         * The profile has reached the goal if the time since the profile started
         * has exceeded the profile's total time.
         *
         * @param t The time since the beginning of the profile.
         */
        bool IsFinished(Duration t) const { return t >= TotalTime(); }

    private:
        /**
         * Returns true if the profile inverted.
         *
         * The profile is inverted if goal position is less than the initial position.
         *
         * @param initial The initial state (usually the current state).
         * @param goal    The desired state when the profile is complete.
         */
        static bool ShouldFlipAcceleration(const State &initial, const State &goal) {
            return initial.position > goal.position;
        }

        // Flip the sign of the velocity and position if the profile is inverted
        State Direct(const State &in) const {
            State result = in;
            result.position *= m_direction;
            result.velocity *= m_direction;
            return result;
        }

        // The direction of the profile, either 1 for forwards or -1 for inverted
        int m_direction{};

        Constraints m_constraints;
        State m_current;
        State m_goal;   // TODO: remove

        Duration m_timeFromStart = 0;
        Duration m_endAccel{};
        Duration m_endFullSpeed{};
        Duration m_endDeccel{};
    };
}  // namespace frc

