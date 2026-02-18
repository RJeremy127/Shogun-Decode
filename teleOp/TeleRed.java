package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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
import org.firstinspires.ftc.teamcode.tools.Tickle;
import org.firstinspires.ftc.teamcode.tools.Turret;
import org.firstinspires.ftc.teamcode.util.Actuation;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="FUCK-RED", group="Linear OpMode")
public class TeleRed extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime flickTimer = new ElapsedTime();
    private Limelight3A limelight;

    // Button edge detection
    private boolean lastSquare = false;
    private boolean lastTriangle2 = false;
    private boolean lastG1Cross = false;
    private boolean lastG2Cross = false;
    private boolean lastG1Triangle = false;
    private boolean lastG1Circle = false;
    private boolean lastDpadDown = false;
    private boolean lastG2Square = false;
    private boolean lastDpadLeft = false;
    private boolean lastTickle = false;
    private boolean lastG2DpadDown = false;
    private boolean lastG2DpadUp = false;
    private boolean lastG2DpadLeft = false;
    private boolean lastG2DpadRight = false;
    private boolean lastG2Options = false;
    private boolean lastG2Share = false;

    // State tracking
    private boolean isTrack = false;
    private boolean isFlicked = false;
    private boolean isSpinningUp = false;
    private boolean isBlocking = false;
    private double targetFlywheelVelocity = 0;
    private double Tx;
    private double Ty;
    private Point position = new Point(0, 0);
    private boolean autoRetractPending = false;
    private boolean lastBlock = true;
    private boolean previousLastBlock = false;
    private static final double RETRACT_DELAY_MS = 2000;

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

    // Shooting mode & auto-transfer
    private boolean shootingMode = false;
    private boolean transferActive = false;
    private int transferStage = 0;
    private int flickCount = 0;
    private int originalDetectedBalls = 0;
    private ElapsedTime transferTimer = new ElapsedTime();
    private boolean transferTimerStarted = false;
    private static final double TRANSFER_TIMEOUT_MS = 5000;

    // Pattern shoot system
    private String[] currentPattern = {"G", "P", "P"};
    private boolean patternSelected = false;
    private boolean patternShootEnabled = false;

    // Flywheel equation mode
    private boolean flywheelEquationEnabled = false;
    private boolean shooterRumbled = false;
    private double lastSeenTx = 0;

    // Turret manual control
    private boolean turretManualActive = false;

    // Timing for wraparound and flywheel dt
    private double lastTime = 0;

    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, new Pose(0, 0, 0), telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        JohnLimeLight.switchToObelisk(limelight);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        limelight.start();

        for (int i = 0; i < 3; i++) ballSlots[i] = null;

        waitForStart();
        limelight.pipelineSwitch(9);
        runtime.reset();
        lastTime = runtime.seconds();
        Turret.setRobotHeading(0);

        while (opModeIsActive()) {
            double currentTime = runtime.seconds();
            double dt = currentTime - lastTime;
            lastTime = currentTime;
            dt = Math.max(dt, 1e-3);

            // Update odometry for heading feedback
            Actuation.otto.updateOdometry();

            // Field-centric turret compensation
            Turret.compensateRotation(Actuation.otto.getPose().getR());

            isFlicked = Tickle.getStatus();
            LLResult llresult = limelight.getLatestResult();

            Flywheel.update();
            Sorter.update();

            // Turret wraparound (takes priority over tracking/manual)
            boolean wrapping = Turret.updateWraparound(dt);

            if (llresult != null && llresult.isValid()) {
                List<LLResultTypes.FiducialResult> results = llresult.getFiducialResults();
                Pose3D botpose = llresult.getBotpose();
                position = JohnLimeLight.getPosition(botpose);
                Tx = llresult.getTx();
                Ty = llresult.getTy();
                if (isTrack && !wrapping) { Turret.track(Tx, Ty); }
            }

            // Auto-retract flickers after shooting
            if (autoRetractPending && flickTimer.milliseconds() >= RETRACT_DELAY_MS) {
                Tickle.retract();
                autoRetractPending = false;
                lastBlock = true;
            }

            // Intake outtake pulse timer
            if (runtime.milliseconds() > intakePauseUntil) {
                intakeActive = true;
            }

            // Ball color detection & auto-sort
            updateBallTracking();

            // Auto-transfer state machine
            if (transferActive) {
                runTransfer();
            }

            // Flywheel at-speed rumble
            if (targetFlywheelVelocity > 0 && Flywheel.isAtSpeed(50) && !shooterRumbled) {
                gamepad1.rumble(1.0, 1.0, 1000);
                shooterRumbled = true;
            }

            pad1(wrapping);
            pad2(wrapping, dt);

            // Telemetry
            telemetry.addData("Shooting Mode", shootingMode
                    ? "READY (" + flickCount + "/" + originalDetectedBalls + ")" : "COLLECT");
            if (shootingMode) {
                telemetry.addData("Flick Order", formatShootingOrder());
            } else {
                telemetry.addData("Intake Order", formatBallSlots());
            }
            telemetry.addData("Pattern", patternShootEnabled
                    ? "ON: " + String.join(", ", currentPattern)
                    : (patternSelected ? "OFF: " + String.join(", ", currentPattern) : "NONE"));
            telemetry.addData("Field-Centric", Turret.isFieldCentricEnabled() ? "ON" : "OFF");
            telemetry.addData("Flywheel Eq", flywheelEquationEnabled ? "ON" : "OFF");
            if (Turret.isWraparoundActive()) {
                telemetry.addData("Turret", "WRAPPING");
            } else {
                telemetry.addData("Turret", Turret.getPosition() + " ticks");
            }
            telemetry.addData("Balls", detectedBalls + "/3");
            telemetry.addData("Flywheel", (int) Flywheel.getVelocity() + " / " + (int) targetFlywheelVelocity);
            telemetry.addData("Sorter", Sorter.getPosition() + " → " + Sorter.getTargetPosition());
            telemetry.addData("Intake", intakeActive ? (intakeToggled ? "ON" : "OFF") : "PULSE");
            // Limelight tracking telemetry
            LLResult llTelemetry = limelight.getLatestResult();
            boolean llValid = llTelemetry != null && llTelemetry.isValid();
            telemetry.addData("Limelight", llValid ? "VALID" : "NO TARGET");
            telemetry.addData("Tracking", isTrack ? "ON" : "OFF");
            telemetry.addData("Tx", "%.2f", Tx);
            telemetry.addData("Ty", "%.2f", Ty);
            if (isTrack) {
                telemetry.addData("Turret Aligned", Turret.isInDeadzone(Tx) ? "YES" : "NO");
            }
            telemetry.update();
        }
    }

    // gamepad 1 (Driver) controls:
    // Left Stick: Drive (strafe)
    // Right Stick: Turn
    // Cross (X): Toggle intake
    // Triangle: Enter shooting mode
    // Circle: Start shoot transfer
    // Square: Add unknown ball + advance spindexer
    // Dpad Left: Reverse spindexer 1 step
    // Dpad Right: Outtake
    // Left Bumper: Turret manual left
    // Right Bumper: Turret manual right
    // Left Trigger: Spindexer manual CCW (direct power)
    // Right Trigger: Spindexer manual CW (direct power)
    public void pad1(boolean wrapping) {
        // Drive
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        Actuation.drive(axial, lateral, yaw);

        // --- Cross (X): Toggle intake ---
        boolean currentG1Cross = gamepad1.cross;
        if (currentG1Cross && !lastG1Cross) {
            intakeToggled = !intakeToggled;
        }
        lastG1Cross = currentG1Cross;

        // --- Dpad Right: Outtake (also g2 circle) ---
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

        // --- Triangle: Enter shooting mode ---
        boolean currentTriangle = gamepad1.triangle;
        if (currentTriangle && !lastG1Triangle && !shootingMode && !transferActive) {
            shootingMode = true;
            Sorter.enterShootingMode();
            rotateForShooting();
            gamepad1.rumble(1.0, 1.0, 500);
        }
        lastG1Triangle = currentTriangle;

        // --- Circle: Start shoot transfer ---
        boolean currentCircle = gamepad1.circle;
        if (currentCircle && !lastG1Circle && shootingMode && !transferActive) {
            if (detectedBalls == 0) {
                gamepad1.rumble(0.5, 0.5, 200);
            } else {
                transferActive = true;
                transferStage = 0;
                flickCount = 0;
                originalDetectedBalls = detectedBalls;
                transferTimerStarted = false;
            }
        }
        lastG1Circle = currentCircle;

        // --- Square: Add unknown ball + advance spindexer ---
        boolean currentSquare = gamepad1.square;
        if (currentSquare && !lastDpadLeft && !shootingMode && !transferActive) {
            if (detectedBalls < 3) {
                addBall("?");
                detectedBalls++;
            }
            Sorter.turn(1);
            manualSpinUsed = true;
        }
        lastDpadLeft = currentSquare;

        // --- Dpad Left: Reverse spindexer 1 step ---
        boolean g1left = gamepad1.dpad_left;
        if (g1left && !lastG2DpadLeft && !transferActive) {
            Sorter.turn(-1);
            manualSpinUsed = true;
        }
        lastG2DpadLeft = g1left;

        // --- Bumpers: Manual turret control (only when not tracking and not wrapping) ---
        if (!wrapping && !isTrack) {
            if (gamepad1.right_bumper) {
                Turret.manualTurn(-0.2);
            } else if (gamepad1.left_bumper) {
                Turret.manualTurn(0.2);
            } else if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                if (turretManualActive) {
                    Turret.syncAfterManual();
                    turretManualActive = false;
                }
            }
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                turretManualActive = true;
            }
        }

        // --- Triggers: Spindexer manual direct power ---
        if (gamepad1.right_trigger > 0.1) {
            Sorter.setManualPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            Sorter.setManualPower(-gamepad1.left_trigger);
        } else if (Sorter.isManualMode()) {
            // Zero-reset on release: sync target to current position
            Sorter.syncTarget();
        }
    }

    // gamepad 2 controls:
    // Square: Field-centric turret toggle
    // Triangle: Toggle turret tracking (Limelight)
    // Cross (X): Toggle flywheel equation on/off
    // Circle (O): Outtake
    // Dpad Down: Toggle pattern shoot
    // Dpad Left: Motif 21 (G, P, P)
    // Dpad Up: Motif 22 (P, G, P)
    // Dpad Right: Motif 23 (P, P, G)
    // Bumpers: Manual turret control
    // Right Trigger: Spindex clockwise
    // Left Trigger: Spindex counterclockwise
    // Share: Close range flywheel preset (1250)
    // Options: Long range flywheel preset (1450)
    public void pad2(boolean wrapping, double dt) {

        // --- Square: Field-centric turret toggle ---
        if (gamepad2.square && !lastSquare) {
            boolean nowEnabled = !Turret.isFieldCentricEnabled();
            Turret.setFieldCentricEnabled(nowEnabled);
            if (nowEnabled) {
                Turret.setRobotHeading(Actuation.otto.getPose().getR());
                gamepad2.rumbleBlips(1);
            } else {
                gamepad2.rumbleBlips(2);
            }
            lastSquare = true;
        }
        if (!gamepad2.square) {
            lastSquare = false;
        }

        // --- Triangle: Toggle turret tracking ---
        if (gamepad2.triangle && !lastTriangle2) {
            isTrack = !isTrack;
            if (!isTrack) {
                Turret.resetPID();
                Turret.stop();
            }
            lastTriangle2 = true;
        }
        if (!gamepad2.triangle) {
            lastTriangle2 = false;
        }

        // --- Cross (X): Toggle flywheel equation on/off ---
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

        // --- Circle (O): Outtake ---
        if (gamepad2.circle) {
            Intake.intakeBall(-1.0);
        }

        // --- Dpad Down: Toggle pattern shoot ---
        boolean g2DpadDown = gamepad2.dpad_down;
        if (g2DpadDown && !lastDpadDown) {
            patternShootEnabled = !patternShootEnabled;
            gamepad2.rumble(0.3, 0.3, 150);
        }
        lastDpadDown = g2DpadDown;

        // --- Dpad Left: Motif 21 (G, P, P) ---
        boolean g2DpadLeft = gamepad2.dpad_left;
        if (g2DpadLeft && !lastG2DpadLeft) {
            currentPattern = new String[]{"G", "P", "P"};
            patternSelected = true;
            gamepad2.rumble(0.3, 0.3, 150);
        }
        lastG2DpadLeft = g2DpadLeft;

        // --- Dpad Up: Motif 22 (P, G, P) ---
        boolean g2DpadUp = gamepad2.dpad_up;
        if (g2DpadUp && !lastG2DpadUp) {
            currentPattern = new String[]{"P", "G", "P"};
            patternSelected = true;
            gamepad2.rumble(0.3, 0.3, 150);
        }
        lastG2DpadUp = g2DpadUp;

        // --- Dpad Right: Motif 23 (P, P, G) ---
        boolean g2DpadRight = gamepad2.dpad_right;
        if (g2DpadRight && !lastG2DpadRight) {
            currentPattern = new String[]{"P", "P", "G"};
            patternSelected = true;
            gamepad2.rumble(0.3, 0.3, 150);
        }
        lastG2DpadRight = g2DpadRight;

        // --- Bumpers: Manual turret control (only when not tracking and not wrapping) ---
        if (!wrapping && !isTrack) {
            if (gamepad2.right_bumper) {
                Turret.manualTurn(-0.2);
            } else if (gamepad2.left_bumper) {
                Turret.manualTurn(0.2);
            } else if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                if (turretManualActive) {
                    Turret.syncAfterManual();
                    turretManualActive = false;
                }
            }
            if (gamepad2.right_bumper || gamepad2.left_bumper) {
                turretManualActive = true;
            }
        }

        // --- Right Trigger: Spindex clockwise ---
        if (gamepad2.right_trigger > 0.2 && !Sorter.isBusy()) {
            Sorter.turn(1);
            manualSpinUsed = true;
        }

        // --- Left Trigger: Spindex counterclockwise ---
        if (gamepad2.left_trigger > 0.2 && !Sorter.isBusy()) {
            Sorter.turn(-1);
            manualSpinUsed = true;
        }

        // --- Share: Close range flywheel preset (1250) ---
        boolean g2Share = gamepad2.share;
        if (g2Share && !lastG2Share && !flywheelEquationEnabled) {
            targetFlywheelVelocity = Flywheel.closeRangeVelocity;
            Flywheel.setTargetVelocity(targetFlywheelVelocity);
            shooterRumbled = false;
            isSpinningUp = true;
            gamepad2.rumble(0.3, 0.3, 150);
        }
        lastG2Share = g2Share;

        // --- Options: Long range flywheel preset (1450) ---
        boolean g2Options = gamepad2.options;
        if (g2Options && !lastG2Options && !flywheelEquationEnabled) {
            targetFlywheelVelocity = Flywheel.longRangeVelocity;
            Flywheel.setTargetVelocity(targetFlywheelVelocity);
            shooterRumbled = false;
            isSpinningUp = true;
            gamepad2.rumble(0.3, 0.3, 150);
        }
        lastG2Options = g2Options;

        // --- Flywheel equation: compute target from tx ---
        if (flywheelEquationEnabled) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                lastSeenTx = result.getTx();
            }
            targetFlywheelVelocity = Flywheel.calculateTargetVelocityFromTx(lastSeenTx);
            Flywheel.setTargetVelocity(targetFlywheelVelocity);
            isSpinningUp = true;
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
        if (intakeToggled && !shootingMode && !transferActive && !manualSpinUsed && sorterAtPosition && intakeActive) {
            if (currentColorDetected && !ballDetectedLastLoop && detectedBalls < 3) {
                addBall(currentColor);
                detectedBalls++;
                Sorter.turn(1);
                ballAdded = true;

                if (detectedBalls == 3) {
                    shootingMode = true;
                    Sorter.enterShootingMode();
                    rotateForShooting();
                    gamepad1.rumble(1.0, 1.0, 500);
                }
            }
        }

        // Outtake pulse: briefly pause intake after ball detected
        if (ballAdded) {
            intakePauseUntil = runtime.milliseconds() + OUTTAKE_PULSE_MS;
            intakeActive = false;
        }

        ballDetectedLastLoop = currentColorDetected;
    }

    private void runTransfer() {
        // Timeout guard
        if (transferTimerStarted && transferTimer.milliseconds() >= TRANSFER_TIMEOUT_MS) {
            transferActive = false;
            shootingMode = false;
            Sorter.exitShootingMode();
            clearBalls();
            gamepad1.rumble(1.0, 1.0, 300);
            return;
        }

        switch (transferStage) {
            case 0: // Flick up
                Tickle.flick();
                if (Tickle.isAtMax()) {
                    if (!transferTimerStarted) {
                        transferTimer.reset();
                        transferTimerStarted = true;
                    }
                    if (transferTimer.milliseconds() >= Tickle.flickUpWaitMs) {
                        transferStage = 1;
                        transferTimerStarted = false;
                    }
                }
                break;

            case 1: // Retract
                Tickle.retract();
                if (Tickle.isAtMin()) {
                    if (!transferTimerStarted) {
                        transferTimer.reset();
                        transferTimerStarted = true;
                    }
                    if (transferTimer.milliseconds() >= Tickle.flickDownWaitMs) {
                        flickCount++;
                        transferTimerStarted = false;

                        // Shift ball slots after each flick
                        for (int i = 0; i < 2; i++) {
                            ballSlots[i] = ballSlots[i + 1];
                        }
                        ballSlots[2] = null;
                        detectedBalls--;

                        if (flickCount >= originalDetectedBalls) {
                            // Transfer complete
                            transferActive = false;
                            shootingMode = false;
                            Sorter.exitShootingMode();
                            clearBalls();
                            intakeToggled = false;
                            gamepad1.rumble(0.3, 0.3, 200);
                        } else {
                            // Advance sorter then flick again
                            transferStage = 2;
                        }
                    }
                }
                break;

            case 2: // Advance sorter
                Sorter.turn(1);
                transferStage = 3;
                break;

            case 3: // Wait for sorter to reach position
                if (!Sorter.isBusy()) {
                    transferStage = 0;
                    transferTimerStarted = false;
                }
                break;
        }
    }

    private void rotateForShooting() {
        // Physical rotation: move first ball to firing position
        if (detectedBalls == 3) {
            String temp = ballSlots[2];
            ballSlots[2] = ballSlots[1];
            ballSlots[1] = ballSlots[0];
            ballSlots[0] = temp;
        } else if (detectedBalls == 2) {
            ballSlots[0] = ballSlots[2];
            ballSlots[2] = null;
        } else if (detectedBalls == 1) {
            ballSlots[0] = ballSlots[2];
            ballSlots[2] = null;
        }

        // Pattern matching (only with 3 balls and pattern shoot enabled)
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
                    // Rotate left by 1
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
                // Wrong ball composition
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

    private String formatShootingOrder() {
        if (detectedBalls == 0) return "No balls";
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < detectedBalls; i++) {
            if (ballSlots[i] != null) {
                sb.append(ballSlots[i]);
            } else {
                sb.append("?");
            }
            if (i < detectedBalls - 1) sb.append(" → ");
        }
        return sb.toString();
    }
}
