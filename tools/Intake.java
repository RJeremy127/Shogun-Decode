package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Intake {
    private static DcMotor intake;
    private static ElapsedTime runtime = new ElapsedTime();

    public static void init(HardwareMap map) {
        intake = map.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static void intakeBall(double power) {
        intake.setPower(power);
    }
    public static void intakeBallColor(double power) {
       while (Color.getColor() == null) {
           intake.setPower(power);
       }
       intake.setPower(0);
       //turn sorter
    }
    public static void intake(double power) {
        if (!Sorter.isFull()) {
            while (Color.getColor() == null) {
                intake.setPower(power);
            }
            intake.setPower(0);
            //turn sorter
            Sorter.turn(1);
            Sorter.updatePorts(Color.getColor());
        }
        else {
            intake.setPower(0);
        }
    }

    public static boolean isBusy() {
        return intake.isBusy();
    }
}
