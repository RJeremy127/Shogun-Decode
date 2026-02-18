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

public abstract class AutoBase extends LinearOpMode {

    protected Limelight3A limelight;
    protected double Tx = 0;
    protected double Ty = 0;

    //init auto
    protected void initAuto(Pose startPose, int pipeline) {
        Actuation.setup(hardwareMap, startPose, telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(pipeline);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * Aim turret at Limelight target until turret is in deadzone.
     * Keeps flywheel and sorter PID alive during tracking.
     */
    protected void aimTurret() {
        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && timeout.seconds() < 3.0) {
            Flywheel.update();
            Sorter.update();
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Tx = result.getTx();
                Ty = result.getTy();
                if (!Turret.autoTrack(Tx, Ty)) {
                    break; // In deadzone, turret is aimed
                }
            } else {
                // No valid result, nudge turret to search
                Turret.turn(-1);
            }
        }
    }

    /**
     * Aim turret and shoot one ball.
     * 1. Aim turret via Limelight
     * 2. Spin flywheel to calculated target velocity
     * 3. Wait until at speed
     * 4. Flick, wait, retract, blockBall
     */
    protected void aimAndShoot() {
        aimTurret();

        // Set flywheel velocity based on Ty
        double targetVelocity = Flywheel.calculateTargetVelocity(Ty);
        Flywheel.setTargetVelocity(targetVelocity);

        // Wait for flywheel to reach speed
        ElapsedTime timeout = new ElapsedTime();
        while (opModeIsActive() && !Flywheel.isAtSpeed(20) && timeout.seconds() < 3.0) {
            Flywheel.update();
            Sorter.update();
            // Keep tracking while spinning up
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Tx = result.getTx();
                Ty = result.getTy();
                Turret.autoTrack(Tx, Ty);
            }
        }

        // Fire
        Tickle.flick();
        sleep(300);
        Tickle.retract();
        sleep(200);
        Tickle.blockBall();
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
