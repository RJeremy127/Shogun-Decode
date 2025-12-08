package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

@TeleOp(name = "Turntable Test")
public class TurntableTest extends LinearOpMode {
    DcMotor t;
    public void runOpMode() {
        t = hardwareMap.get(DcMotor.class, "turntable");
        t.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        t.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            int position = t.getCurrentPosition();
            telemetry.addData("Encoder Position", position);
            telemetry.update();
        }
    }
}
