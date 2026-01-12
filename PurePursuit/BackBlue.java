package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.tools.Flywheel;
import org.firstinspires.ftc.teamcode.tools.Tickle;
import org.firstinspires.ftc.teamcode.tools.Turret;
import org.firstinspires.ftc.teamcode.util.Actuation;

@Autonomous(name = "BackBlue")
public class BackBlue extends LinearOpMode {
    private Limelight3A limelight;
    private double tx, ty;

    @Override
    public void runOpMode() {
        // Initialize all systems
        Actuation.setup(hardwareMap, new Pose(0, 0, 0), telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        limelight.start();
        waitForStart();

        // Switch to blue goal pipeline (8 = blue, 9 = red)
        limelight.pipelineSwitch(8);

        // Step 1: Go forward
        telemetry.addData("Status", "Moving forward");
        telemetry.update();

        Pose[] forwardPath = new Pose[]{new Pose(0, 15, 0)};
        Route forwardRoute = new Route(forwardPath);
        forwardRoute.run(0.5, 0.2);

        // Step 2: Track AprilTag
        telemetry.addData("Status", "Tracking target");
        telemetry.update();

        trackTarget();

        // Step 3: Shoot
        telemetry.addData("Status", "Shooting");
        telemetry.update();

        shoot();

        telemetry.addData("Status", "Complete");
        telemetry.update();

        sleep(2000);
    }

    private void trackTarget() {
        LLResult result = limelight.getLatestResult();

        // Search for target by rotating turret if not found
        while (opModeIsActive() && (result == null || !result.isValid())) {
            Turret.turn(1);
            result = limelight.getLatestResult();
            sleep(50);
        }

        // Track until aligned
        boolean tracking = true;
        while (opModeIsActive() && tracking) {
            result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                tx = result.getTx();
                ty = result.getTy();
                tracking = Turret.autoTrack(tx, ty);

                telemetry.addData("TX", tx);
                telemetry.addData("TY", ty);
                telemetry.addData("Tracking", tracking);
                telemetry.update();
            }
            sleep(20);
        }
    }

    private void shoot() {
        // Calculate and set flywheel velocity based on distance
        double targetVelocity = Flywheel.calculateTargetVelocity(ty);
        Flywheel.setTargetVelocity(targetVelocity);

        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.update();

        // Spin up flywheel with PID updates
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < 3000) {
            Flywheel.update();

            // Check if flywheel is at speed
            if (Flywheel.isAtSpeed(50)) {
                break;
            }
            sleep(10);
        }

        // Fire
        Tickle.flick();
        sleep(500);

        // Retract and stop
        Tickle.blockBall();
        Flywheel.stop();
    }
}
