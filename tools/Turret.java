package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Turret {
    private static DcMotor motor;
    private static int maxTicks = 2150;
    private static int ticks;
    private static double motorPower;
    private static int target = 0;

    public static void init(DcMotor mt, int ticksPerRevolution, double power) {
        motor = mt;
        ticks = ticksPerRevolution;
        motorPower = power;
    }
    public static void turn(int ticks) {
        target += ticks;
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(motorPower);
    }
    //need AprilTag class
    public static void track(double tx) {
        double kP = 0.015;
        double power = -tx*kP;
        power = Math.max(-motorPower, Math.min(motorPower, power));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (Math.abs(tx) < 3.0) {
            motor.setPower(0);
        }
        else {
            motor.setPower(power);
        }
    }
}
