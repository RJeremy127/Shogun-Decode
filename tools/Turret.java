package org.firstinspires.ftc.teamcode.tools;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {
    private static DcMotor motor;
    private static int maxTicks = 1538;
    private static int ticks = 700;
    private static double motorPower = 0.8;
    private static int target = 0;

    // PD controller state
    private static double lastTx = 0;
    private static ElapsedTime timer = new ElapsedTime();
    private static double lastTime = 0;

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
        motor.setPower(motorPower);
    }
    //need AprilTag class
    public static void track(double tx) {
        // PD controller gains - tune these for your turret
        double kP = 0.015;  // Proportional gain (responsiveness)
        double kD = 0.008;  // Derivative gain (dampening/smoothing)

        // Calculate time delta
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;

        // Calculate derivative (rate of change of error)
        double derivative = 0;
        if (dt > 0 && dt < 0.5) {  // Skip invalid or too-large time deltas
            derivative = (tx - lastTx) / dt;
        }

        // PD control: combine proportional and derivative terms
        double power = -(tx * kP + derivative * kD);

        // Clamp power to motor limits
        power = Math.max(-motorPower, Math.min(motorPower, power));

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Deadzone: stop when close enough to target
        if (Math.abs(tx) < 3.0) {
            motor.setPower(0);
        } else {
            motor.setPower(power);
        }

        // Update state for next iteration
        lastTx = tx;
        lastTime = currentTime;
    }
}
