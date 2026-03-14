package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.tools.Color;
import org.firstinspires.ftc.teamcode.tools.Flywheel;
import org.firstinspires.ftc.teamcode.tools.Intake;
import org.firstinspires.ftc.teamcode.tools.Sorter;
import org.firstinspires.ftc.teamcode.tools.Turret;
import org.firstinspires.ftc.teamcode.util.Actuation;
import org.firstinspires.ftc.teamcode.util.MathFunctions;

public abstract class AutoBase extends LinearOpMode {

    protected Limelight3A limelight;
    protected double Tx = 0;
    protected double Ty = 0;
    protected int goalPipeline = 8;

    // Search direction: -1 = left, +1 = right. Set by subclass.
    protected int searchDirection = -1;

    // Turret offset in ticks to compensate for tilted AprilTag. Set by subclass.
    protected int turretOffset = 0;

    protected void initAuto(Pose startPose, int pipeline) {
        goalPipeline = pipeline;
        Actuation.setup(hardwareMap, startPose, telemetry);
        Sorter.preloadBalls("green", "purple", "purple");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(goalPipeline);
        Flywheel.setTargetVelocity(500);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * Aim turret at Limelight target until turret is in deadzone.
     * Turns continuously in searchDirection until the Limelight detects a target,
     * then fine-tracks with PID.
     */
    protected void aimTurret() {
        Turret.TurretTracking.enableTracking = true;
        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && timeout.seconds() < 1.0) {
            Flywheel.update(0.02);
            Sorter.updateSlew();
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Tx = result.getTx();
                Ty = result.getTy();
                Turret.update(0.02, Tx, false, false);
                if (Math.abs(Tx) <= Turret.TurretTracking.txDeadzone) {
                    break;
                }
            } else {
                // No target — search by spinning turret directly (bypass update which zeroes power)
                Turret.manualTurn(searchDirection * 0.15);
            }
        }
        Turret.TurretTracking.enableTracking = false;
        Turret.stop();
    }

    /**
     * Aim turret and shoot one ball.
     * 1. Aim turret with Limelight (search if needed)
     * 2. Spin flywheel to Ty-based target velocity
     * 3. Wait until at speed
     * 4. Flick, wait, retract
     */
    protected void aimAndShoot() {
        aimTurret();

        Flywheel.setTargetVelocity(1425);

        // Wait for flywheel to reach speed, keep tracking
        Turret.TurretTracking.enableTracking = true;
        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && !Flywheel.isAtSpeed(20) && timeout.seconds() < 5.0) {
            Flywheel.update(0.02);
            Sorter.updateSlew();
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Tx = result.getTx();
                Ty = result.getTy();
                Turret.update(0.02, Tx, false, false);
            } else {
                // No target — keep searching
                Turret.manualTurn(searchDirection * 0.15);
            }
        }
        Turret.TurretTracking.enableTracking = false;
        // Force transfer regardless of ball count
        Sorter.TransferSettings.preventZeroBallTransfer = false;
        Sorter.startTransfer();
        ElapsedTime transferTimeout = new ElapsedTime();
        while (opModeIsActive() && Sorter.isTransferActive() && transferTimeout.seconds() < 7.0) {
            Flywheel.update(0.02);
            Sorter.updateSlew();
            Sorter.checkPendingTransfer();
            Sorter.updateTransferMotor(0);
            Sorter.handleTransfer();
        }
        // Stop transfer motor after shooting
        Sorter.stopTransfer();
    }

    /**
     * Shoot all balls from the sorter.
     * Goes to port 0 (no sorting), does a full rotation transfer to fire everything.
     */
    protected void shootAllThree() {
        // Go to port 0 (no color sorting) and wait for settle
        Sorter.turnToPort(0);
        ElapsedTime settleTimeout = new ElapsedTime();
        while (opModeIsActive() && !Sorter.isSettled() && settleTimeout.seconds() < 2.0) {
            Sorter.updateSlew();
        }

        // Aim and fire (full rotation regardless of ball count)
        aimAndShoot();
        // Flywheel stays at idle 800 RPM — set in initAuto
        Flywheel.setTargetVelocity(800);
    }

    /**
     * Intake 3 balls without color sorting.
     * Runs intake motor and uses Color sensor to detect rising edges.
     * Commits balls as "unknown" (no color sorting).
     */
    protected void intakeThreeBalls() {
        int detectedBalls = 0;
        boolean lastDetected = false;
        Intake.intakeBall(1);

        Sorter.stopTransfer();
        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && detectedBalls < 3 && timeout.seconds() < 8.0) {
            Flywheel.update(0.02);
            Sorter.updateSlew();

            String color = Color.getColor();
            boolean detected = (color != null);

            if (detected && !lastDetected) {
                detectedBalls++;
                Sorter.commitBall("unknown");
                telemetry.addData("Balls Detected", detectedBalls);
                telemetry.update();
            }

            lastDetected = detected;
        }

        Intake.stop();
    }

    /**
     * Drive to a target pose while keeping PID controllers alive.
     * Uses Route with opModeIsActive() safety check.
     */
    protected void driveToWithUpdates(Pose target, double moveSpeed, double turnSpeed) {
        Route route = new Route(new Pose[]{target});
        route.run(moveSpeed, turnSpeed, this);
    }

    /**
     * Drive along a row of balls while intaking.
     * Drives from the current position toward endPose while running the intake
     * and detecting balls with the color sensor. Stops when 3 balls are collected,
     * the robot reaches the end of the row, or the timeout expires.
     *
     * @param endPose     End of the ball row to drive toward
     * @param moveSpeed   Drive speed (0-1)
     * @param turnSpeed   Turn speed (0-1)
     * @param timeoutSec  Max time before giving up
     */
    protected void driveAndIntake(Pose endPose, double moveSpeed, double turnSpeed, double timeoutSec) {

        int detectedBalls = 0;
        boolean lastDetected = false;
        Intake.intakeBall(0.8);
        Sorter.stopTransfer();

        ElapsedTime timeout = new ElapsedTime();

        while (opModeIsActive() && detectedBalls < 3 && timeout.seconds() < timeoutSec) {
            // Drive toward end of row
            Actuation.otto.updateOdometry();
            Pose robotPose = Actuation.otto.getPose();
            Pose worldPose = new Pose(
                    robotPose.getX() + RobotMovement.initPos.getX(),
                    robotPose.getY() + RobotMovement.initPos.getY(),
                    robotPose.getR() + RobotMovement.initPos.getR());
            RobotMovement.goToPosition(endPose, moveSpeed, turnSpeed);

            // Check if we've reached the end of the row
            boolean atEnd = MathFunctions.distance(worldPose.getPoint(), endPose.getPoint()) < 2
                    && Math.abs(MathFunctions.angleWrap(endPose.getR() - worldPose.getR())) < Math.toRadians(3);

            Flywheel.update(0.02);
            Sorter.updateSlew();

            // Ball detection (no color sorting)
            String color = Color.getColor();
            boolean detected = (color != null);

            if (detected && !lastDetected) {
                detectedBalls++;
                Sorter.commitBall("unknown");
                telemetry.addData("Balls Detected", detectedBalls);
                telemetry.update();
            }
            lastDetected = detected;

            // Stop driving if we reached the end but keep intaking if balls remain
            if (atEnd) {
                Actuation.drive(0, 0, 0);
            }
        }

        Actuation.drive(0, 0, 0);
        Intake.stop();
    }

    /**
     * Wait for human player to feed balls via intake with color detection.
     * Similar to intakeThreeBalls but with a longer timeout for human player.
     */
    protected void waitForHumanFeed(int ballCount) {
        int detectedBalls = 0;
        boolean lastDetected = false;
        Intake.intakeBall(0.8);
        Sorter.stopTransfer();

        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && detectedBalls < ballCount && timeout.seconds() < 15.0) {
            Flywheel.update(0.02);
            Sorter.updateSlew();

            String color = Color.getColor();
            boolean detected = (color != null);

            if (detected && !lastDetected) {
                detectedBalls++;
                Sorter.commitBall("unknown");
                telemetry.addData("Balls Fed", detectedBalls + "/" + ballCount);
                telemetry.update();
            }

            lastDetected = detected;
        }

        Intake.stop();
    }
}
