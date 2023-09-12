#include "ProfiledPIDController.h"
#include <spdlog/spdlog.h>

static constexpr auto kDt = 0.05;

void testTrapezoidProfile() {
    frc::TrapezoidProfile::Constraints constraints{750, 500};  // max vel: 0.75m/s, max acc: 1m/ss
    frc::TrapezoidProfile::State goal{3000, 0}; // final dist: 3m, vel: 0
    frc::TrapezoidProfile::State state;4

    frc::TrapezoidProfile profile{constraints};
    for (int i = 0; i < 280; ++i) {
        state = profile.Calculate(kDt, goal, state);
        spdlog::info("idx[{}] time[{:0.2f}] pos[{:0.2f}/{}], vel[{:0.2f} mm/s]",
                     i, i * kDt, state.position, goal.position, state.velocity);
        if (std::abs(goal.position - state.position) == 0) {
            spdlog::set_level(spdlog::level::off);
        }
    }
}

void testProfiledPID() {
    frc::TrapezoidProfile::Constraints constraints{750, 500};  // max vel: 0.75m/s, max acc: 1m/ss
    frc::ProfiledPIDController controller(1.0, 1.0, 0.6, constraints, kDt);
    frc::TrapezoidProfile::State goal{3000, 0}; // final dist: 3m, vel: 0
    frc::ProfiledPIDController::Distance measurement = 0.0;

    for (int i = 0; i < 300; ++i) {
        double output = controller.Calculate(measurement, goal);
        measurement += output * kDt;
        spdlog::info("<ProfiledPID> idx[{}] time[{:0.2f}] pos[{:0.2f}/{}], vel[{:0.2f} mm/s]\n",
                     i, i * kDt, measurement, goal.position, output);
        if (std::abs(goal.position - measurement) == 0) {
            spdlog::set_level(spdlog::level::off);
        }
    }
}

int main(int argc, char *argv[]) {
    spdlog::set_level(spdlog::level::info);
    // testTrapezoidProfile();
    testProfiledPID();

}