package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest")
public class ServoTest extends LinearOpMode{
    private Servo test;
    public void runOpMode() {
        test = hardwareMap.get(Servo.class, "test");
        waitForStart();
        while (opModeIsActive()) {
            //test.setPosition(0);
            test.setPosition(0);
            //test.setPosition(0);
        }
    }
}
