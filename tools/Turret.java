package org.firstinspires.ftc.teamcode.tools;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {
    private static DcMotor motor;
    //PID
    private static double kP = 0.01;
    private static double kI = 0.0001;
    private static double kD = 0.0008;

    private static int maxTicks = -199;
    private static int minTicks = 230;
    private static double motorPower = 0.6;
    private static int target = 0;

    // PID controller state
    private static double lastTx = 0;
    private static double integralSum = 0;
    private static ElapsedTime timer = new ElapsedTime();
    private static double lastTime = 0;
    // Tracking deadzone (tx values within this range are considered "on target")
    private static double minTx = -0.5;
    private static double maxTx = 0.5;
    private static double integralMax = 0.3;

    // Field-centric heading compensation
    // Gear ratio 8:27 (27/8 = 3.375 reduction), motor TPR ~384.5
    // ticksPerRadian = 384.5 * (27.0/8.0) / (2 * Math.PI) ≈ 206.5
    public static double ticksPerRadian = 206.5;
    private static double lastRobotHeading = 0;
    private static boolean fieldCentricEnabled = true;
    private static double headingCompensationPower = 0; // feedforward for tracking mode

    public static void init(HardwareMap map) {
        motor = map.get(DcMotor.class, "turnTable");
        timer.reset();
        lastTime = 0;
        lastTx = 0;
        lastRobotHeading = 0;
        headingCompensationPower = 0;
    }

    public static void setRobotHeading(double heading) {
        lastRobotHeading = heading;
        headingCompensationPower = 0;
    }

    public static void compensateRotation(double currentHeading) {
        if (!fieldCentricEnabled) return;

        double deltaHeading = currentHeading - lastRobotHeading;
        lastRobotHeading = currentHeading;

        if (Math.abs(deltaHeading) < 1e-6) {
            headingCompensationPower = 0;
            return;
        }

        int deltaTicks = (int) (-deltaHeading * ticksPerRadian);

        // Adjust the RUN_TO_POSITION target (used when not tracking)
        target += deltaTicks;
        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setTargetPosition(target);
        }

        // Store compensation as feedforward power for tracking mode
        // Scale: convert tick delta to proportional power
        headingCompensationPower = -deltaHeading * ticksPerRadian * kP;
        headingCompensationPower = Math.max(-motorPower, Math.min(motorPower, headingCompensationPower));
    }

    public static void setFieldCentricEnabled(boolean enabled) {
        fieldCentricEnabled = enabled;
    }

    public static boolean isFieldCentricEnabled() {
        return fieldCentricEnabled;
    }
    public static void turn(int ticks) {
        target += ticks;
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Only power motor when within safe bounds (-1150 to -240)
        //if (motor.getCurrentPosition() >= rightBound && motor.getCurrentPosition() <= leftBound) {
        motor.setPower(motorPower);
        //} else {
         //   motor.setPower(0);  // Stop motor when out of bounds
        //}
    }

    public static int getPosition() {
        return motor.getCurrentPosition();
    }

    public static void stop() {
        motor.setPower(0.0);
    }
    //need AprilTag class
    public static void track(double tx, double ty) {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        dt = Math.max(dt, 1e-3);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Deadzone: stop when tx is within tolerance
        if (tx >= minTx && tx <= maxTx) {
            motor.setPower(0);
            integralSum = 0;
            lastTx = 0;
            return;
        }

        double error = -tx;

        // Reset integral on sign change (prevents overshoot)
        if (lastTx != 0 && Math.signum(error) != Math.signum(lastTx)) {
            integralSum = 0;
        }

        // Integral with anti-windup
        integralSum += error * dt;
        integralSum = Math.max(-integralMax, Math.min(integralMax, integralSum));

        // Derivative
        double derivative = (error - lastTx) / dt;

        // PID output
        double power = (kP * error) + (kI * integralSum) + (kD * derivative);
        power = Math.max(-motorPower, Math.min(motorPower, power));

        // Bounds checking
        int turretPos = motor.getCurrentPosition();
        if (turretPos <= minTicks && power < 0) {
            power = 0;
            integralSum = 0;
        } else if (turretPos >= maxTicks && power > 0) {
            power = 0;
            integralSum = 0;
        }

        // Add heading feedforward for field-centric compensation
        power += headingCompensationPower;
        power = Math.max(-motorPower, Math.min(motorPower, power));
        headingCompensationPower = 0; // consume the feedforward

        motor.setPower(-power);
        lastTx = error;
    }
    public static boolean autoTrack(double tx, double ty) {
        // Use track() method for consistent PID behavior
        track(tx, ty);
        // Return true if still tracking (not in deadzone)
        return !(tx >= minTx && tx <= maxTx);
    }

    public static void resetPID() {
        integralSum = 0;
        lastTx = 0;
        lastTime = timer.seconds();
    }

    public static boolean isInDeadzone(double tx) {
        return tx >= minTx && tx <= maxTx;
    }
}
