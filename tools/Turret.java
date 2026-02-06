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

    public static void init(HardwareMap map) {
        motor = map.get(DcMotor.class, "turnTable");
        timer.reset();
        lastTime = 0;
        lastTx = 0;
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
