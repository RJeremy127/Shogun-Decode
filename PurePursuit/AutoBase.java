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
import org.firstinspires.ftc.teamcode.tools.Tickle;
import org.firstinspires.ftc.teamcode.tools.Turret;
import org.firstinspires.ftc.teamcode.util.Actuation;
import org.firstinspires.ftc.teamcode.util.MathFunctions;

public abstract class AutoBase extends LinearOpMode {

    protected Limelight3A limelight;
    protected double Tx = 0;
    protected double Ty = 0;

    // Goal position for pre-aiming (field coordinates, inches)
    protected double goalX = 0;
    protected double goalY = 0;

    // Sweep search constants
    private static final int SWEEP_STEP = 5;
    private static final int SWEEP_RANGE = 50;

    protected void initAuto(Pose startPose, int pipeline, double goalX, double goalY) {
        Actuation.setup(hardwareMap, startPose, telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(pipeline);
        this.goalX = goalX;
        this.goalY = goalY;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * Pre-aim the turret toward the goal using odometry.
     * Calculates the field-relative angle to the goal and moves the turret
     * so the AprilTag is near the center of the Limelight's FOV.
     */
    protected void preAimAtGoal() {
        Pose robot = Actuation.otto.getPose();
        double dx = goalX - robot.getX();
        double dy = goalY - robot.getY();
        double fieldAngle = Math.atan2(dx, dy);
        double turretAngle = MathFunctions.angleWrap(fieldAngle - robot.getR());
        int targetTicks = (int) (turretAngle * Turret.ticksPerRadian);
        int deltaTicks = targetTicks - Turret.getPosition();
        Turret.turn(deltaTicks);
    }

    /**
     * Aim turret at Limelight target until turret is in deadzone.
     * Pre-aims using odometry first, then fine-tracks with Limelight.
     * If no target is found, sweeps turret back and forth to search.
     */
    protected void aimTurret() {
        preAimAtGoal();

        int sweepCenter = Turret.getPosition();
        int sweepDirection = 1;
        int sweepOffset = 0;

        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && timeout.seconds() < 3.0) {
            Flywheel.update();
            Sorter.update();
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Tx = result.getTy();
                Ty = result.getTx();
                if (!Turret.autoTrack(Tx, Ty)) {
                    break; // In deadzone, turret is aimed
                }
            } else {
                // Sweep search: oscillate turret to find the tag
                sweepOffset += SWEEP_STEP * sweepDirection;
                if (Math.abs(sweepOffset) >= SWEEP_RANGE) {
                    sweepDirection = -sweepDirection;
                }
                Turret.turn(SWEEP_STEP * sweepDirection);
            }
        }
    }

    /**
     * Aim turret and shoot one ball.
     * 1. Pre-aim turret using odometry
     * 2. Fine-track with Limelight (sweep if needed)
     * 3. Spin flywheel to target velocity
     * 4. Wait until at speed
     * 5. Flick, wait, retract
     */
    protected void aimAndShoot() {
        aimTurret();

        // Set flywheel velocity
        double targetVelocity = 1450;
        Flywheel.setTargetVelocity(targetVelocity);

        // Wait for flywheel to reach speed
        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && !Flywheel.isAtSpeed(20) && timeout.seconds() < 3.0) {
            Flywheel.update();
            Sorter.update();
            // Keep tracking while spinning up
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Tx = result.getTy();
                Ty = result.getTx();
                Turret.autoTrack(Tx, Ty);
            }
        }

        // Fire
        Tickle.flick();
        sleep(300);
        Tickle.retract();
    }

    /**
     * Shoot all 3 balls from the sorter.
     * Enters shooting mode, fires 3 times with sorter advancement, then exits shooting mode.
     */
    protected void shootAllThree() {
        Sorter.enterShootingMode();

        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            aimAndShoot();

            if (i < 2) {
                // Advance sorter to next ball
                Sorter.turn(1);
                // Wait for sorter to reach position
                ElapsedTime sorterTimeout = new ElapsedTime();
                while (opModeIsActive() && Sorter.isBusy() && sorterTimeout.seconds() < 2.0) {
                    Sorter.update();
                    Flywheel.update();
                }
            }
        }

        Sorter.exitShootingMode();
        Flywheel.setTargetVelocity(0);
    }

    /**
     * Intake 3 balls with color detection.
     * Runs intake motor and uses Color sensor to detect rising edges.
     * Advances sorter on each ball detection.
     */
    protected void intakeThreeBalls() {
        int detectedBalls = 0;
        boolean lastDetected = false;
        Intake.intakeBall(0.8);

        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && detectedBalls < 3 && timeout.seconds() < 8.0) {
            Flywheel.update();
            Sorter.update();

            String color = Color.getColor();
            boolean detected = (color != null);

            // Rising edge: no ball last loop, ball this loop
            if (detected && !lastDetected) {
                detectedBalls++;
                Sorter.updatePorts(color);
                if (detectedBalls < 3) {
                    Sorter.turn(1);
                }
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
     * Wait for human player to feed balls via intake with color detection.
     * Similar to intakeThreeBalls but with a longer timeout for human player.
     */
    protected void waitForHumanFeed(int ballCount) {
        int detectedBalls = 0;
        boolean lastDetected = false;
        Intake.intakeBall(0.8);

        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && detectedBalls < ballCount && timeout.seconds() < 15.0) {
            Flywheel.update();
            Sorter.update();

            String color = Color.getColor();
            boolean detected = (color != null);

            if (detected && !lastDetected) {
                detectedBalls++;
                Sorter.updatePorts(color);
                if (detectedBalls < ballCount) {
                    Sorter.turn(1);
                }
                telemetry.addData("Balls Fed", detectedBalls + "/" + ballCount);
                telemetry.update();
            }

            lastDetected = detected;
        }

        Intake.stop();
    }
}
