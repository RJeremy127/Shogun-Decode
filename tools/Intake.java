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

    public static void intakeBall(double power, double spinTime) {
       double startTime = runtime.seconds();
       while (runtime.seconds() < (startTime+spinTime)) {
          intake.setPower(power);
       }
       intake.setPower(0);
    }
    public static void intakeBall(double power, Color sensor) {
       while (sensor.getColor() == null) {
           intake.setPower(power);
       }
       intake.setPower(0);
       //turn sorter
    }
    public static void intakeBall(double power) {
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
}
