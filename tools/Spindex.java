package org.firstinspires.ftc.teamcode.tools;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "SPINNER")
public class Spindex extends LinearOpMode {

    @Configurable
    public static class Config {
        public static double motorPower = 0.2;
        public static int stepTicks = 83;
    }

    DcMotor motor;

    int target = 0;
    boolean lastTriangle = false;



    @Override
    public void runOpMode() {

        motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        target = 0;
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TelemetryManager.TelemetryWrapper panels = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        waitForStart();

        while (opModeIsActive()) {

            boolean triangle = gamepad1.y;

            if (triangle && !lastTriangle) {
                target += Config.stepTicks;
                motor.setTargetPosition(target);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(Config.motorPower);

            }

            lastTriangle = triangle;

            panels.addData("Current", motor.getCurrentPosition());
            panels.addData("Target", target);
            panels.addData("Power", Config.motorPower);
            panels.addData("Step Size", Config.stepTicks);
            panels.update();
        }
    }
}
