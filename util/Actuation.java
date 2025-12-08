package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.RobotConfigNameable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.control.Odometry;
import org.firstinspires.ftc.teamcode.datatypes.Pose;

public class Actuation {
    private static DcMotor[] motors;
    public static Odometry otto;

    public static void setup(HardwareMap map, Pose startPose, Telemetry t) { // Initialize other things as well
         motors = HardwareMapper.getMotors(map);
         otto = new Odometry(new DcMotor[] {motors[1], motors[2], motors[0]});
         otto.resetEncoders();
        RobotMovement.setup(startPose, t);
    }

    public static void drive(double axial, double lateral, double yaw) {
        double max;
        double leftFrontPower  = axial + lateral+yaw;
        double rightFrontPower = axial - lateral-yaw;
        double leftBackPower   = axial - lateral+yaw;
        double rightBackPower  = axial + lateral-yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        motors[3].setPower(leftFrontPower);
        motors[2].setPower(rightFrontPower);
        motors[1].setPower(leftBackPower);
        motors[0].setPower(rightBackPower);
    }


}
