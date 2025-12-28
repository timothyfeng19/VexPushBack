#include "robot-config.h"
#include "vex.h"

void dsr(double targetDistance, int timeoutMs, double leniency = 0.005) {
    const double maxVolt = 5.0;   // top speed
    const double minVolt = 1.0;   // minimum voltage to overcome friction
    const double slowDist = 6.0;  // start slowing down within this distance

    auto calcVolt = [&](double error) -> double {
        if (fabs(error) <= leniency) return 0;
        double volt = (fabs(error) / slowDist) * maxVolt;
        if (volt > maxVolt) volt = maxVolt;
        if (volt < minVolt) volt = minVolt;
        return (error > 0) ? volt : -volt;
    };

    int elapsed = 0;

    // Main approach with timeout
    while (true) {
        double leftCurrent = left_dist.objectDistance(inches);
        double rightCurrent = right_dist.objectDistance(inches);

        double leftError = targetDistance - leftCurrent;
        double rightError = targetDistance - rightCurrent;

        // Stop if both within leniency
        if (fabs(leftError) <= leniency && fabs(rightError) <= leniency) {
            left_chassis.stop();
            right_chassis.stop();
            break;
        }

        left_chassis.spin(forward, calcVolt(leftError), voltageUnits::volt);
        right_chassis.spin(forward, calcVolt(rightError), voltageUnits::volt);

        task::sleep(20);
        elapsed += 20;

        if (elapsed >= timeoutMs) {  // timeout reached
            left_chassis.stop();
            right_chassis.stop();
            break;
        }
    }

    // Final corrections (each side independently), also respecting timeout
    bool leftDone = false;
    bool rightDone = false;

    while ((!leftDone || !rightDone) && elapsed < timeoutMs) {
        double leftCurrent = left_dist.objectDistance(inches);
        double rightCurrent = right_dist.objectDistance(inches);

        double leftError = targetDistance - leftCurrent;
        double rightError = targetDistance - rightCurrent;

        if (fabs(leftError) <= leniency) {
            left_chassis.stop();
            leftDone = true;
        } else {
            left_chassis.spin(forward, calcVolt(leftError), voltageUnits::volt);
        }

        if (fabs(rightError) <= leniency) {
            right_chassis.stop();
            rightDone = true;
        } else {
            right_chassis.spin(forward, calcVolt(rightError), voltageUnits::volt);
        }

        task::sleep(20);
        elapsed += 20;
    }

    // Ensure motors are fully stopped at the end
    left_chassis.stop();
    right_chassis.stop();
}