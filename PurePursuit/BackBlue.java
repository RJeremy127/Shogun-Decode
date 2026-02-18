package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.datatypes.Pose;

@Autonomous(name = "FUCK-BackBlue", group = "Auto")
public class BackBlue extends AutoBase {

    @Override
    public void runOpMode() {
        // Pipeline 8 = blue goals
        initAuto(new Pose(0, 0, 0), 8);

        waitForStart();

        telemetry.addData("Phase", "Shooting preloaded balls");
        telemetry.update();
        shootAllThree();

        telemetry.addData("Phase", "Driving to loading zone");
        telemetry.update();
        driveToWithUpdates(new Pose(0, -10, 0), 0.5, 0.2); // TODO: TUNE position

        telemetry.addData("Phase", "Waiting for human player feed (1)");
        telemetry.update();
        waitForHumanFeed(3);

        telemetry.addData("Phase", "Returning to shoot position");
        telemetry.update();
        driveToWithUpdates(new Pose(0, 0, 0), 0.5, 0.2); // TODO: TUNE position

        telemetry.addData("Phase", "Shooting cycle 2");
        telemetry.update();
        shootAllThree();

        // Drive to shooting position
        telemetry.addData("Phase", "Returning to shoot position");
        telemetry.update();
        driveToWithUpdates(new Pose(0, 0, 0), 0.5, 0.2); // TODO: TUNE position

        telemetry.addData("Phase", "Shooting cycle 3");
        telemetry.update();
        shootAllThree();
    }
}
