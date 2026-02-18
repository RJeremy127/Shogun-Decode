package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.datatypes.Pose;

@Autonomous(name = "FUCK-BackRed", group = "Auto")
public class BackRed extends AutoBase {

    @Override
    public void runOpMode() {
        // Pipeline 9 = red goals
        initAuto(new Pose(0, 0, 0), 9);

        waitForStart();

        // === Cycle 1: Shoot 3 preloaded balls ===
        telemetry.addData("Phase", "Shooting preloaded balls");
        telemetry.update();
        shootAllThree();

        // === Cycle 2: Drive to loading zone, wait for human feed, shoot 3 ===
        telemetry.addData("Phase", "Driving to loading zone");
        telemetry.update();
        driveToWithUpdates(new Pose(0, 10, 0), 0.5, 0.2); // TODO: TUNE position (mirrored Y)

        telemetry.addData("Phase", "Waiting for human player feed (1)");
        telemetry.update();
        waitForHumanFeed(3);

        // Drive to shooting position
        telemetry.addData("Phase", "Returning to shoot position");
        telemetry.update();
        driveToWithUpdates(new Pose(0, 0, 0), 0.5, 0.2); // TODO: TUNE position

        telemetry.addData("Phase", "Shooting cycle 2");
        telemetry.update();
        shootAllThree();

        // === Cycle 3: Return to loading zone, wait for human feed, shoot 3 ===
        telemetry.addData("Phase", "Driving to loading zone");
        telemetry.update();
        driveToWithUpdates(new Pose(0, 10, 0), 0.5, 0.2); // TODO: TUNE position (mirrored Y)

        telemetry.addData("Phase", "Waiting for human player feed (2)");
        telemetry.update();
        waitForHumanFeed(3);

        // Drive to shooting position
        telemetry.addData("Phase", "Returning to shoot position");
        telemetry.update();
        driveToWithUpdates(new Pose(0, 0, 0), 0.5, 0.2); // TODO: TUNE position

        telemetry.addData("Phase", "Shooting cycle 3");
        telemetry.update();
        shootAllThree();

        telemetry.addData("Phase", "DONE - 9 balls scored");
        telemetry.update();
    }
}
