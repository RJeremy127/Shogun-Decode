package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tools.Flywheel;

@TeleOp(name = "Flywheel PID Tuner", group = "Test")
public class FlywheelPIDTuner extends LinearOpMode {

    private double kP = 0.0001;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 0.0001;

    private double targetVelocity = 0;
    private final double TARGET_INCREMENT = 100;
    private final double MAX_VELOCITY = 3000;

    // Which gain is currently selected (0=P, 1=I, 2=D, 3=F)
    private int selectedGain = 0;
    private final String[] GAIN_NAMES = {"kP", "kI", "kD", "kF"};

    // Increment amounts for fine/coarse tuning
    private double increment = 0.00001;
    private final double[] INCREMENT_OPTIONS = {0.000001, 0.00001, 0.0001, 0.001, 0.01};
    private int incrementIndex = 1;

    // Button debouncing
    private ElapsedTime buttonTimer = new ElapsedTime();
    private final double DEBOUNCE_TIME = 0.2;

    @Override
    public void runOpMode() {
        Flywheel.init(hardwareMap);

        telemetry.addLine("=== Flywheel PID Tuner ===");
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPAD UP/DOWN: Select gain (P/I/D/F)");
        telemetry.addLine("  DPAD LEFT/RIGHT: Adjust selected gain");
        telemetry.addLine("  LEFT/RIGHT BUMPER: Change increment size");
        telemetry.addLine("  LEFT/RIGHT TRIGGER: Set target velocity");
        telemetry.addLine("  A: Start flywheel");
        telemetry.addLine("  B: Stop flywheel");
        telemetry.addLine("  Y: Reset all gains to defaults");
        telemetry.update();

        waitForStart();
        buttonTimer.reset();

        while (opModeIsActive()) {
            handleGainSelection();
            handleGainAdjustment();
            handleIncrementChange();
            handleVelocityControl();
            handleFlywheelControl();

            // Update flywheel PID
            Flywheel.setGains(kP, kI, kD, kF);
            Flywheel.update();

            // Display telemetry
            displayTelemetry();
        }

        Flywheel.stop();
    }

    private void handleGainSelection() {
        if (buttonTimer.seconds() < DEBOUNCE_TIME) return;

        if (gamepad1.dpad_up) {
            selectedGain = (selectedGain - 1 + 4) % 4;
            buttonTimer.reset();
        } else if (gamepad1.dpad_down) {
            selectedGain = (selectedGain + 1) % 4;
            buttonTimer.reset();
        }
    }

    private void handleGainAdjustment() {
        if (buttonTimer.seconds() < DEBOUNCE_TIME) return;

        double delta = 0;
        if (gamepad1.dpad_right) {
            delta = increment;
            buttonTimer.reset();
        } else if (gamepad1.dpad_left) {
            delta = -increment;
            buttonTimer.reset();
        }

        if (delta != 0) {
            switch (selectedGain) {
                case 0: kP = Math.max(0, kP + delta); break;
                case 1: kI = Math.max(0, kI + delta); break;
                case 2: kD = Math.max(0, kD + delta); break;
                case 3: kF = Math.max(0, kF + delta); break;
            }
        }
    }

    private void handleIncrementChange() {
        if (buttonTimer.seconds() < DEBOUNCE_TIME) return;

        if (gamepad1.right_bumper) {
            incrementIndex = Math.min(incrementIndex + 1, INCREMENT_OPTIONS.length - 1);
            increment = INCREMENT_OPTIONS[incrementIndex];
            buttonTimer.reset();
        } else if (gamepad1.left_bumper) {
            incrementIndex = Math.max(incrementIndex - 1, 0);
            increment = INCREMENT_OPTIONS[incrementIndex];
            buttonTimer.reset();
        }
    }

    private void handleVelocityControl() {
        // Right trigger increases, left trigger decreases
        if (gamepad1.right_trigger > 0.5 && buttonTimer.seconds() > DEBOUNCE_TIME) {
            targetVelocity = Math.min(targetVelocity + TARGET_INCREMENT, MAX_VELOCITY);
            buttonTimer.reset();
        } else if (gamepad1.left_trigger > 0.5 && buttonTimer.seconds() > DEBOUNCE_TIME) {
            targetVelocity = Math.max(targetVelocity - TARGET_INCREMENT, 0);
            buttonTimer.reset();
        }
    }

    private void handleFlywheelControl() {
        if (gamepad1.a) {
            Flywheel.setTargetVelocity(targetVelocity);
        } else if (gamepad1.b) {
            Flywheel.stop();
            targetVelocity = 0;
        } else if (gamepad1.y && buttonTimer.seconds() > DEBOUNCE_TIME) {
            // Reset to defaults
            kP = 0.0001;
            kI = 0.0;
            kD = 0.0;
            kF = 0.0001;
            buttonTimer.reset();
        }
    }

    private void displayTelemetry() {
        telemetry.addLine("=== GAINS (DPAD U/D to select, L/R to adjust) ===");
        telemetry.addData(selectedGain == 0 ? "> kP" : "  kP", formatGain(kP));
        telemetry.addData(selectedGain == 1 ? "> kI" : "  kI", formatGain(kI));
        telemetry.addData(selectedGain == 2 ? "> kD" : "  kD", formatGain(kD));
        telemetry.addData(selectedGain == 3 ? "> kF" : "  kF", formatGain(kF));

        telemetry.addLine();
        telemetry.addData("Increment (LB/RB)", formatGain(increment));

        telemetry.addLine();
        telemetry.addLine("=== VELOCITY ===");
        telemetry.addData("Target (triggers)", "%.0f ticks/s", targetVelocity);
        telemetry.addData("Actual", "%.0f ticks/s", Flywheel.getVelocity());
        telemetry.addData("Error", "%.0f ticks/s", Flywheel.getError());
        telemetry.addData("At Speed?", Flywheel.isAtSpeed(50) ? "YES" : "NO");

        telemetry.addLine();
        telemetry.addLine("A=Start | B=Stop | Y=Reset gains");

        telemetry.update();
    }

    private String formatGain(double value) {
        if (value == 0) return "0";
        if (value < 0.0001) return String.format("%.7f", value);
        if (value < 0.001) return String.format("%.6f", value);
        if (value < 0.01) return String.format("%.5f", value);
        return String.format("%.4f", value);
    }
}
