package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.datatypes.Point;
import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.tools.Color;
import org.firstinspires.ftc.teamcode.tools.Flywheel;
import org.firstinspires.ftc.teamcode.tools.Intake;
import org.firstinspires.ftc.teamcode.tools.JohnLimeLight;
import org.firstinspires.ftc.teamcode.tools.Sorter;
import org.firstinspires.ftc.teamcode.tools.Turret;
import org.firstinspires.ftc.teamcode.util.Actuation;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FRICK-BLUE", group="Linear OpMode")
public class TeleBlue extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Limelight3A limelight;
    private double lastTime = 0;

    // Button edge detection
    private boolean lastTriangle2 = false;
    private boolean lastG1Cross = false;
    private boolean lastG2Cross = false;
    private boolean lastG1Triangle = false;
    private boolean lastG1Circle = false;
    private boolean lastG1Square = false;
    private boolean lastG1DpadLeft = false;
    private boolean lastG2DpadDown = false;
    private boolean lastG2DpadUp = false;
    private boolean lastG2DpadLeft = false;
    private boolean lastG2DpadRight = false;
    private boolean lastG2Options = false;
    private boolean lastG2Share = false;
    private boolean lastG2RightTrigger = false;
    private boolean lastG2LeftTrigger = false;

    // State tracking
    private double targetFlywheelVelocity = 0;
    private double Tx;
    private double Ty;
    private boolean limelightValid = false;
    private Point position = new Point(0, 0);

    // Intake toggle & outtake pulse
    private boolean intakeToggled = false;
    private boolean intakeActive = true;
    private double intakePauseUntil = 0;
    private static final double OUTTAKE_PULSE_MS = 400;

    // Ball slot tracking
    private int detectedBalls = 0;
    private String[] ballSlots = new String[3];
    private boolean ballDetectedLastLoop = false;
    private boolean manualSpinUsed = false;

    // Shooting mode
    private boolean shootingMode = false;

    // Pattern shoot system
    private String[] currentPattern = {"G", "P", "P"};
    private boolean patternSelected = false;
    private boolean patternShootEnabled = false;

    // Flywheel equation mode
    private boolean flywheelEquationEnabled = false;
    private boolean shooterRumbled = false;

    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, new Pose(0, 0, 0), telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        limelight.start();

        for (int i = 0; i < 3; i++) ballSlots[i] = null;

        waitForStart();
        limelight.pipelineSwitch(8);
        runtime.reset();
        lastTime = runtime.seconds();

        while (opModeIsActive()) {
            double now = runtime.seconds();
            double dt = Math.max(now - lastTime, 1e-3);
            lastTime = now;

            Actuation.otto.updateOdometry();

            LLResult llresult = limelight.getLatestResult();

            Flywheel.update(dt);
            Sorter.updateSlew();
            Sorter.checkPendingTransfer();

            limelightValid = llresult != null && llresult.isValid();
            if (limelightValid) {
                Pose3D botpose = llresult.getBotpose();
                position = JohnLimeLight.getPosition(botpose);
                Tx = llresult.getTx();
                Ty = llresult.getTy();
            } else {
                Tx = 0;
                Ty = 0;
            }
            Turret.update(dt, limelightValid ? Tx : null,
                    gamepad1.left_bumper || gamepad2.left_bumper,
                    gamepad1.right_bumper || gamepad2.right_bumper);

            // G1 triggers: manual transfer motor power (auto power applied during active transfer)
            double g1TriggerPower = 0;
            if (gamepad1.right_trigger > 0.5) g1TriggerPower = 0.2;
            else if (gamepad1.left_trigger > 0.5) g1TriggerPower = -0.2;
            Sorter.updateTransferMotor(g1TriggerPower);

            // Transfer state machine
            int transferResult = Sorter.handleTransfer();
            if (transferResult == 1 || transferResult == 2) {
                shootingMode = false;
                clearBalls();
                intakeToggled = false;
                shooterRumbled = false;
                flywheelEquationEnabled = false;
                targetFlywheelVelocity = 0;
                Flywheel.setTargetVelocity(0);
                gamepad1.rumble(0.3, 0.3, 200);
            }

            if (runtime.milliseconds() > intakePauseUntil) {
                intakeActive = true;
            }

            updateBallTracking();

            if (targetFlywheelVelocity > 0 && Flywheel.isAtSpeed(50) && !shooterRumbled) {
                gamepad1.rumble(1.0, 1.0, 1000);
                shooterRumbled = true;
            }

            pad1();
            pad2();

            telemetry.addData("Shooting Mode", shootingMode ? "READY" : "COLLECT");
            if (shootingMode) {
                telemetry.addData("Shoot Order", Sorter.formatShootingOrder());
            } else {
                telemetry.addData("Intake Order", formatBallSlots());
            }
            telemetry.addData("Pattern", patternShootEnabled
                    ? "ON: " + String.join(", ", currentPattern)
                    : (patternSelected ? "OFF: " + String.join(", ", currentPattern) : "NONE"));
            telemetry.addData("Flywheel Eq", flywheelEquationEnabled ? "ON" : "OFF");
            telemetry.addData("Turret", Turret.getPosition() + " ticks");
            telemetry.addData("Balls", detectedBalls + "/3");
            telemetry.addData("Flywheel", (int) Flywheel.getVelocity() + " / " + (int) targetFlywheelVelocity);
            telemetry.addData("Transfer", Sorter.isTransferActive() ? "SHOOTING" : Sorter.isTransferPending() ? "PENDING" : "IDLE");
            telemetry.addData("Spindexer", "pos=" + String.format("%.3f", Sorter.getCurrentPos()));
            telemetry.addData("Intake", intakeActive ? (intakeToggled ? "ON" : "OFF") : "PULSE");
            telemetry.addData("Limelight", limelightValid ? "VALID" : "NO TARGET");
            telemetry.addData("Tracking", Turret.isTrackingEnabled() ? "ON" : "OFF");
            telemetry.addData("Tx", "%.2f", Tx);
            telemetry.addData("Ty", "%.2f", Ty);
            if (Turret.isTrackingEnabled()) {
                telemetry.addData("Turret Aligned", Turret.isInDeadzone(Tx) ? "YES" : "NO");
            }
            telemetry.update();
        }
    }

    // Gamepad 1:
    // Left Stick: Drive (strafe)    Right Stick: Turn
    // Cross: Toggle intake           Triangle: Toggle shooting mode
    // Circle: Start transfer         Square: Add unknown ball
    // Dpad Left: Reverse spindexer   Dpad Right: Outtake
    // Bumpers: Turret manual (handled in main loop via Turret.update)
    // Triggers: Transfer motor manual power
    public void pad1() {
        Actuation.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        boolean currentG1Cross = gamepad1.cross;
        if (currentG1Cross && !lastG1Cross) {
            intakeToggled = !intakeToggled;
        }
        lastG1Cross = currentG1Cross;

        boolean outtakeActive = gamepad1.dpad_right || gamepad2.circle;
        if (outtakeActive) {
            Intake.intakeBall(-1.0);
        } else if (!intakeActive) {
            Intake.intakeBall(0);
        } else if (intakeToggled && !shootingMode) {
            Intake.intakeBall(1.0);
        } else {
            Intake.stop();
        }

        boolean currentTriangle = gamepad1.triangle;
        if (currentTriangle && !lastG1Triangle && !Sorter.isTransferActive()) {
            if (!shootingMode) {
                shootingMode = true;
                shooterRumbled = false;
                Sorter.rotateForShooting(false);
                rotateForShooting();
                gamepad1.rumble(1.0, 1.0, 500);
            } else {
                shootingMode = false;
                shooterRumbled = false;
                targetFlywheelVelocity = 0;
                Flywheel.setTargetVelocity(0);
                flywheelEquationEnabled = false;
                gamepad1.rumble(0.3, 0.3, 200);
            }
        }
        lastG1Triangle = currentTriangle;

        boolean currentCircle = gamepad1.circle;
        if (currentCircle && !lastG1Circle) {
            if (shootingMode && !Sorter.isTransferActive() && !Sorter.isTransferPending()
                    && detectedBalls > 0 && targetFlywheelVelocity > 0) {
                Sorter.startTransfer();
            }
        }
        lastG1Circle = currentCircle;

        boolean currentSquare = gamepad1.square;
        if (currentSquare && !lastG1Square && !shootingMode && !Sorter.isTransferActive()) {
            if (detectedBalls < 3) {
                addBall("?");
                detectedBalls++;
                Sorter.updatePorts("?");
            }
            Sorter.turn(1);
            manualSpinUsed = true;
        }
        lastG1Square = currentSquare;

        boolean g1left = gamepad1.dpad_left;
        if (g1left && !lastG1DpadLeft && !Sorter.isTransferActive()) {
            Sorter.turn(-1);
            manualSpinUsed = true;
        }
        lastG1DpadLeft = g1left;

    }

    // Gamepad 2:
    // Triangle: Toggle turret tracking   Cross: Toggle flywheel equation
    // Circle: Outtake                    Dpad Down: Toggle pattern shoot
    // Dpad Left/Up/Right: Pattern select Left/Right Bumper: Turret manual
    // Triggers: Spindexer step CW/CCW    Share: Close range preset
    // Options: Long range preset
    public void pad2() {
        if (gamepad2.triangle && !lastTriangle2) {
            Turret.toggleTracking();
            lastTriangle2 = true;
        }
        if (!gamepad2.triangle) {
            lastTriangle2 = false;
        }

        boolean g2Cross = gamepad2.cross;
        if (g2Cross && !lastG2Cross) {
            flywheelEquationEnabled = !flywheelEquationEnabled;
            if (flywheelEquationEnabled) {
                shooterRumbled = false;
                gamepad2.rumbleBlips(1);
            } else {
                targetFlywheelVelocity = 0;
                Flywheel.setTargetVelocity(0);
                shooterRumbled = false;
                gamepad2.rumbleBlips(2);
            }
        }
        lastG2Cross = g2Cross;

        boolean g2DpadDown = gamepad2.dpad_down;
        if (g2DpadDown && !lastG2DpadDown) {
            if (!patternSelected) {
                gamepad2.rumble(1.0, 0.0, 300);
            } else {
                patternShootEnabled = !patternShootEnabled;
                gamepad2.rumbleBlips(patternShootEnabled ? 1 : 2);
            }
        }
        lastG2DpadDown = g2DpadDown;

        boolean g2DpadLeft = gamepad2.dpad_left;
        if (g2DpadLeft && !lastG2DpadLeft) {
            currentPattern = new String[]{"G", "P", "P"};
            patternSelected = true;
            gamepad2.rumble(0.3, 0.3, 150);
        }
        lastG2DpadLeft = g2DpadLeft;

        boolean g2DpadUp = gamepad2.dpad_up;
        if (g2DpadUp && !lastG2DpadUp) {
            currentPattern = new String[]{"P", "G", "P"};
            patternSelected = true;
            gamepad2.rumble(0.3, 0.3, 150);
        }
        lastG2DpadUp = g2DpadUp;

        boolean g2DpadRight = gamepad2.dpad_right;
        if (g2DpadRight && !lastG2DpadRight) {
            currentPattern = new String[]{"P", "P", "G"};
            patternSelected = true;
            gamepad2.rumble(0.3, 0.3, 150);
        }
        lastG2DpadRight = g2DpadRight;

        boolean g2RightTrigger = gamepad2.right_trigger > 0.2;
        if (g2RightTrigger && !lastG2RightTrigger && !Sorter.isBusy()) {
            Sorter.advanceSpindexer();
            manualSpinUsed = true;
        }
        lastG2RightTrigger = g2RightTrigger;

        boolean g2LeftTrigger = gamepad2.left_trigger > 0.2;
        if (g2LeftTrigger && !lastG2LeftTrigger && !Sorter.isBusy()) {
            Sorter.reverseSpindexer();
            manualSpinUsed = true;
        }
        lastG2LeftTrigger = g2LeftTrigger;

        boolean g2Share = gamepad2.share;
        if (g2Share && !lastG2Share && !flywheelEquationEnabled) {
            targetFlywheelVelocity = Flywheel.FlywheelPID.closeRangeVelocity;
            Flywheel.setTargetVelocity(targetFlywheelVelocity);
            shooterRumbled = false;
            gamepad2.rumble(0.3, 0.3, 150);
        }
        lastG2Share = g2Share;

        boolean g2Options = gamepad2.options;
        if (g2Options && !lastG2Options && !flywheelEquationEnabled) {
            targetFlywheelVelocity = Flywheel.FlywheelPID.longRangeVelocity;
            Flywheel.setTargetVelocity(targetFlywheelVelocity);
            shooterRumbled = false;
            gamepad2.rumble(0.3, 0.3, 150);
        }
        lastG2Options = g2Options;

        if (flywheelEquationEnabled && limelightValid) {
            targetFlywheelVelocity = Flywheel.calculateTargetVelocity(Ty);
            Flywheel.setTargetVelocity(targetFlywheelVelocity);
        }
    }

    private void updateBallTracking() {
        String currentColor = Color.getColor();
        boolean currentColorDetected = currentColor != null;
        boolean sorterAtPosition = !Sorter.isBusy();

        if (sorterAtPosition && manualSpinUsed) {
            manualSpinUsed = false;
            ballDetectedLastLoop = false;
        }

        boolean ballAdded = false;
        if (intakeToggled && !shootingMode && !Sorter.isTransferActive() && !manualSpinUsed && sorterAtPosition && intakeActive) {
            if (currentColorDetected && !ballDetectedLastLoop && detectedBalls < 3) {
                addBall(currentColor);
                detectedBalls++;
                Sorter.updatePorts(currentColor);
                Sorter.turn(1);
                ballAdded = true;

                if (detectedBalls == 3) {
                    shootingMode = true;
                    shooterRumbled = false;
                    Sorter.rotateForShooting(false);
                    rotateForShooting();
                    gamepad1.rumble(1.0, 1.0, 500);
                }
            }
        }

        if (ballAdded) {
            intakePauseUntil = runtime.milliseconds() + OUTTAKE_PULSE_MS;
            intakeActive = false;
        }

        ballDetectedLastLoop = currentColorDetected;
    }

    private void rotateForShooting() {
        if (detectedBalls == 3) {
            String temp = ballSlots[2];
            ballSlots[2] = ballSlots[1];
            ballSlots[1] = ballSlots[0];
            ballSlots[0] = temp;
        } else if (detectedBalls >= 1) {
            ballSlots[0] = ballSlots[2];
            ballSlots[2] = null;
        }

        if (patternShootEnabled && detectedBalls == 3) {
            int pCount = 0, gCount = 0;
            for (int i = 0; i < 3; i++) {
                if ("P".equals(ballSlots[i])) pCount++;
                if ("G".equals(ballSlots[i])) gCount++;
            }

            if (pCount == 2 && gCount == 1) {
                boolean matched = false;
                for (int cycle = 0; cycle < 3; cycle++) {
                    if (ballSlots[0].equals(currentPattern[0]) &&
                        ballSlots[1].equals(currentPattern[1]) &&
                        ballSlots[2].equals(currentPattern[2])) {
                        matched = true;
                        break;
                    }
                    String temp = ballSlots[0];
                    ballSlots[0] = ballSlots[1];
                    ballSlots[1] = ballSlots[2];
                    ballSlots[2] = temp;
                    Sorter.turn(1);
                }
                if (matched) {
                    gamepad2.rumble(0.5, 0.5, 200);
                } else {
                    gamepad1.rumble(0.8, 0.8, 400);
                    gamepad2.rumble(0.8, 0.8, 400);
                }
            } else {
                gamepad1.rumble(0.8, 0.8, 400);
                gamepad2.rumble(0.8, 0.8, 400);
            }
        }
    }

    private void addBall(String color) {
        for (int i = 0; i < 2; i++) {
            ballSlots[i] = ballSlots[i + 1];
        }
        ballSlots[2] = color;
    }

    private void clearBalls() {
        for (int i = 0; i < ballSlots.length; i++) {
            ballSlots[i] = null;
        }
        detectedBalls = 0;
        ballDetectedLastLoop = false;
        Sorter.clearPorts();
    }

    private String formatBallSlots() {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < ballSlots.length; i++) {
            sb.append(ballSlots[i] == null ? "empty" : ballSlots[i]);
            if (i < ballSlots.length - 1) sb.append(", ");
        }
        sb.append("]");
        return sb.toString();
    }
}
