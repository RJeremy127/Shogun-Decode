package org.firstinspires.ftc.teamcode.tools;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private static DcMotor motor;
    private static int maxTicks = 2150;
    private static int ticks = 700;
    private static double motorPower = 0.5;
    private static int target = 0;

    public static void init(HardwareMap map) {
        motor = map.get(DcMotor.class, "turnTable");
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
            gamepad1.rumble(200);
            gamepad1.stopRumble();
            gamepad1.rumble(500);
        }
        else {
            motor.setPower(power);
        }
    }
}
