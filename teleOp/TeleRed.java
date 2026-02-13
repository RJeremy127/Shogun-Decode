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

import java.util.Arrays;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TELE-RED", group="Linear OpMode")
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

    // Intake toggle
    private boolean intakeToggled = false;

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
    private ElapsedTime transferTimer = new ElapsedTime();
    private boolean transferTimerStarted = false;

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
        Turret.setRobotHeading(0);

        while (opModeIsActive()) {
            // Update odometry for heading feedback
            Actuation.otto.updateOdometry();

            // Field-centric turret compensation
            Turret.compensateRotation(Actuation.otto.getPose().getR());

            isFlicked = Tickle.getStatus();
            LLResult llresult = limelight.getLatestResult();

            Flywheel.update();
            Sorter.update();

            if (llresult != null && llresult.isValid()) {
                List<LLResultTypes.FiducialResult> results = llresult.getFiducialResults();
                Pose3D botpose = llresult.getBotpose();
                position = JohnLimeLight.getPosition(botpose);
                Tx = llresult.getTx();
                Ty = llresult.getTy();
                if (isTrack) { Turret.track(Tx, Ty); }
            }

            // Auto-blocking: only call blockBall() when transitioning to enabled state
            if (lastBlock && !previousLastBlock) {
                Tickle.blockBall();
            }
            previousLastBlock = lastBlock;

            // Auto-retract flickers after shooting
            if (autoRetractPending && flickTimer.milliseconds() >= RETRACT_DELAY_MS) {
                Tickle.retract();
                autoRetractPending = false;
                lastBlock = true;
            }

            // Ball color detection & auto-sort
            updateBallTracking();

            // Auto-transfer state machine
            if (transferActive) {
                runTransfer();
            }

            pad1();
            pad2();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Intake", intakeToggled ? "ON" : "OFF");
            telemetry.addData("Shooting Mode", shootingMode ? "READY" : "COLLECT");
            telemetry.addData("Is tracking", isTrack);
            telemetry.addData("Flywheel Target", String.format("%.0f", targetFlywheelVelocity));
            telemetry.addData("Flywheel Ready", isSpinningUp && Flywheel.isAtSpeed(50));
            telemetry.addData("Tx", Tx);
            telemetry.addData("Ty", Ty);
            telemetry.addData("Position", position.toString());
            telemetry.addData("Turret Position", Turret.getPosition());
            telemetry.addData("Ball Slots", formatBallSlots());
            telemetry.addData("Balls", detectedBalls + "/3");
            telemetry.addData("Sorter Pos", Sorter.getPosition());
            telemetry.addData("Sorter Target", Sorter.getTargetPosition());
            telemetry.update();
        }
    }

    // gamepad 1: drive + intake + shooting mode controls
    public void pad1() {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        Actuation.drive(axial, lateral, yaw);

        // Intake toggle (cross on either gamepad)
        boolean currentG1Cross = gamepad1.cross;
        if (currentG1Cross && !lastG1Cross) {
            intakeToggled = !intakeToggled;
        }
        lastG1Cross = currentG1Cross;

        boolean currentG2Cross = gamepad2.cross;
        if (currentG2Cross && !lastG2Cross) {
            intakeToggled = !intakeToggled;
        }
        lastG2Cross = currentG2Cross;

        // Outtake (dpad_right)
        if (gamepad1.dpad_right) {
            Intake.intakeBall(-1.0);
        } else if (intakeToggled && !shootingMode) {
            Intake.intakeBall(1.0);
        } else {
            Intake.stop();
        }

        // Enter shooting mode (triangle) - only if not already shooting or transferring
        boolean currentTriangle = gamepad1.triangle;
        if (currentTriangle && !lastG1Triangle && !shootingMode && !transferActive) {
            shootingMode = true;
            Sorter.enterShootingMode();
        }
        lastG1Triangle = currentTriangle;

        // Start auto-transfer (circle) - only in shooting mode
        boolean currentCircle = gamepad1.circle;
        if (currentCircle && !lastG1Circle && shootingMode && !transferActive) {
            transferActive = true;
            transferStage = 0;
            flickCount = 0;
            transferTimerStarted = false;
        }
        lastG1Circle = currentCircle;

        // Manual sorter controls
        boolean currentSquare = gamepad1.square;
        if (currentSquare && !lastDpadLeft && !shootingMode) {
            Sorter.turn(1);
            manualSpinUsed = true;
        }
        lastDpadLeft = currentSquare;

        // Sorter bumper controls
        if (gamepad1.right_bumper && !Sorter.isBusy()) {
            Sorter.turn(1);
        }
        if (gamepad1.left_bumper && !Sorter.isBusy()) {
            Sorter.turn(-1);
        }
    }

    // gamepad 2: turret + shoot + sorter manual
    public void pad2() {
        // Turret control - toggle tracking mode
        if (gamepad2.square && !lastSquare) {
            isTrack = !isTrack;
            if (!isTrack) {
                Turret.resetPID();
                Turret.stop();
            }
            lastSquare = true;
        }
        if (!gamepad2.square) {
            lastSquare = false;
        }

        // Toggle blockBall/retract with gamepad2.triangle
        if (gamepad2.triangle && !lastTriangle2) {
            if (isBlocking) {
                Tickle.retract();
                isBlocking = false;
                lastBlock = true;
            } else {
                Tickle.blockBall();
                isBlocking = true;
                lastBlock = false;
            }
            lastTriangle2 = true;
        }
        if (!gamepad2.triangle) {
            lastTriangle2 = false;
        }

        // Manual sorter via gamepad2 dpad_down (reverse) and square (forward)
        boolean g2DpadDown = gamepad2.dpad_down;
        if (g2DpadDown && !lastDpadDown && !Sorter.isBusy()) {
            Sorter.turn(-1);
            manualSpinUsed = true;
            if (detectedBalls > 0) {
                // Shift ball slots down
                for (int i = 0; i < 2; i++) {
                    ballSlots[i] = ballSlots[i + 1];
                }
                ballSlots[2] = null;
                detectedBalls--;
                if (shootingMode && detectedBalls < 3) {
                    shootingMode = false;
                    Sorter.exitShootingMode();
                }
                ballDetectedLastLoop = false;
            }
        }
        lastDpadDown = g2DpadDown;

        boolean g2Square = gamepad2.square;
        if (g2Square && !lastG2Square && !shootingMode && !Sorter.isBusy()) {
            Sorter.turn(1);
            manualSpinUsed = true;
        }
        lastG2Square = g2Square;

        // Manual turret control only when not in tracking mode
        if (gamepad2.dpad_right && !isTrack) {
            Turret.turn(5);
        }
        if (gamepad2.dpad_left && !isTrack) {
            Turret.turn(-5);
        }
        if (!isTrack && !gamepad2.square) {
            Turret.turn((int) (-gamepad2.left_stick_x * 10));
        }

        // Flicker control
        if (gamepad2.dpad_down && !lastTickle) {
            if (Tickle.getStatus()) {
                Tickle.retract();
                lastTickle = true;
                autoRetractPending = false;
            } else if (gamepad2.dpad_up) {
                Tickle.flick();
                lastTickle = true;
                autoRetractPending = false;
            }
        }
        if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
            lastTickle = false;
        }

        // Manual flywheel control with left trigger (with auto-flick)
        if (gamepad2.left_trigger > 0.2) {
            double manualPower = 1550;
            Flywheel.setTargetVelocity(manualPower);
            isSpinningUp = true;

            if (Flywheel.isAtSpeed(20) && !autoRetractPending) {
                gamepad2.rumble(2000);
                Tickle.flick();
                autoRetractPending = true;
                flickTimer.reset();
            }
        }
        // Flywheel shooting with PID velocity control and auto-flick
        else if (gamepad2.right_trigger > 0.2) {
            targetFlywheelVelocity = Flywheel.calculateTargetVelocity(Ty);
            Flywheel.setTargetVelocity(targetFlywheelVelocity);
            isSpinningUp = true;

            if (Flywheel.isAtSpeed(20) && !autoRetractPending) {
                gamepad2.rumble(2000);
                Tickle.flick();
                autoRetractPending = true;
                flickTimer.reset();
            }
        }
        // Stop flywheel when both triggers released
        else {
            targetFlywheelVelocity = 0;
            Flywheel.setTargetVelocity(0);
            isSpinningUp = false;
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

        if (intakeToggled && !shootingMode && !transferActive && !manualSpinUsed && sorterAtPosition) {
            if (currentColorDetected && !ballDetectedLastLoop && detectedBalls < 3) {
                addBall(currentColor);
                detectedBalls++;
                Sorter.turn(1);

                if (detectedBalls == 3) {
                    shootingMode = true;
                    Sorter.enterShootingMode();
                }
            }
        }

        ballDetectedLastLoop = currentColorDetected;
    }

    private void runTransfer() {
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

                        if (flickCount >= 3) {
                            // Transfer complete
                            transferActive = false;
                            shootingMode = false;
                            Sorter.exitShootingMode();
                            clearBalls();
                            intakeToggled = false;
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
                }
                break;
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
}
