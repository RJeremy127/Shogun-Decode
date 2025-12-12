package org.firstinspires.ftc.teamcode.tools;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.telemetry.PanelsTelemetry;

@TeleOp(name = "hooded", group = "Tuning")
public class Tunets extends LinearOpMode {

    TelemetryManager.TelemetryWrapper panelsTelemetery = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    @Configurable
    public static class MotorPIDConfig {
        public static double kP = 0.0005;
        public static double kI = 0.0;
        public static double kD = 0.000006;
        public static double kF = 0.000353;
        public static double targetVelocity = 200;
        public static double integralMax = 0.3;
    }
    @Configurable
    public static class TurnTable {
        public static double kP = 0.0005;
        public static double kI = 0.0;
        public static double kD = 0.000006;
        public static double kF = 0.000353;
        public static double targetVelocity = 200;
        public static double integralMax = 0.3;
    }

    @Configurable
    public static class ServoConfig {
        public static double servoSpeed = 0.01;
        public static double minPosition = 0.0;
        public static double maxPosition = 1.0;
    }

    private DcMotorEx motor;
    private Servo servo;
    private double servoPosition = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(servoPosition);

        panelsTelemetery.addData("Status", "Initialized - Ready to start");
        panelsTelemetery.addData("Controls", "Triggers: Move Servo | D-Pad: Motor Speed | A: Reset Integral");
        panelsTelemetery.update();

        waitForStart();
        timer.reset();
        lastTime = timer.seconds();

        while (opModeIsActive()) {
            double currentTime = timer.seconds();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            if (dt == 0)
                dt = 0.02;

            double velocity = motor.getVelocity();
            double error = MotorPIDConfig.targetVelocity - velocity;

            integralSum += error * dt;
            integralSum = Math.max(-MotorPIDConfig.integralMax,
                    Math.min(MotorPIDConfig.integralMax, integralSum));

            double derivative = (error - lastError) / dt;

            double feedForward = MotorPIDConfig.kF * MotorPIDConfig.targetVelocity;

            double output = (MotorPIDConfig.kP * error)
                    + (MotorPIDConfig.kI * integralSum)
                    + (MotorPIDConfig.kD * derivative)
                    + feedForward;

            output = Math.max(-1.0, Math.min(1.0, output));
            motor.setPower(output);
            lastError = error;

            if (gamepad1.right_trigger > 0.1) {
                servoPosition += ServoConfig.servoSpeed * gamepad1.right_trigger;
            }

            if (gamepad1.left_trigger > 0.1) {
                servoPosition -= ServoConfig.servoSpeed * gamepad1.left_trigger;
            }

            servoPosition = Math.max(ServoConfig.minPosition,
                    Math.min(ServoConfig.maxPosition, servoPosition));

            servo.setPosition(servoPosition);

            if (gamepad1.dpad_up) MotorPIDConfig.targetVelocity += 100;
            if (gamepad1.dpad_down) MotorPIDConfig.targetVelocity -= 100;
            if (gamepad1.a) integralSum = 0;


            panelsTelemetery.addData("Servo Position", "%.3f", servoPosition);
            panelsTelemetery.addData("Right Trigger", "%.2f", gamepad1.right_trigger);
            panelsTelemetery.addData("Left Trigger", "%.2f", gamepad1.left_trigger);
            panelsTelemetery.addData("", "");


            panelsTelemetery.addData("Target Velocity", MotorPIDConfig.targetVelocity);
            panelsTelemetery.addData("Current Velocity", velocity);
            panelsTelemetery.addData("Error", error);
            panelsTelemetery.addData("Integral Sum", integralSum);
            panelsTelemetery.addData("Derivative", derivative);
            panelsTelemetery.addData("Motor Power", output);
            panelsTelemetery.addData("dt (seconds)", dt);
            panelsTelemetery.addData("", "");


            panelsTelemetery.addData("kP", MotorPIDConfig.kP);
            panelsTelemetery.addData("kI", MotorPIDConfig.kI);
            panelsTelemetery.addData("kD", MotorPIDConfig.kD);
            panelsTelemetery.addData("kF", MotorPIDConfig.kF);
            panelsTelemetery.update();
        }

        motor.setPower(0);
        servo.setPosition(0.5);
    }
}