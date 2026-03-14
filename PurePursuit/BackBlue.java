package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.datatypes.Pose;

@Autonomous(name = "SHIT-Blue", group = "Auto")
public class BackBlue extends AutoBase {

    private final double radians = Math.toRadians(-90);

    // Shooting position — robot faces goals from here
    Pose shooting = new Pose(4.5, 7.4, 0);

    // Row 1: approach point, then drive-and-intake endpoint
    Pose row1Approach = new Pose(28.37, -4, 0);        // TODO: tune
    Pose row1Turn = new Pose(28.37, -4, radians);
    Pose row1End      = new Pose(30, -30, radians);       // TODO: tune

    // Row 2: approach point, then drive-and-intake endpoint
    Pose row2Approach = new Pose(50, -4, 0);        // TODO: tune
    Pose row2Turn = new Pose(50, -4, radians);
    Pose row2End      = new Pose(50, -30, radians);       // TODO: tune

    double moveSpeed = 0.5;
    double turnSpeed = 0.4;
    double intakeSpeed = 0.25;

    @Override
    public void runOpMode() {
        initAuto(new Pose(0, 0, 0), 8); // pipeline 8 = blue goals
        searchDirection = 1;

        waitForStart();
        // detectAndSetMotif();

        // === Cycle 1: shoot 3 preloaded balls ===
        telemetry.addData("Phase", "Cycle 1 - shooting preloaded");
        telemetry.update();
        driveToWithUpdates(shooting, moveSpeed, turnSpeed);
        shootAllThree();

        // === Cycle 2: intake row 1, return, shoot ===
        telemetry.addData("Phase", "Cycle 2 - intaking row 1");
        telemetry.update();
        driveToWithUpdates(row1Approach, moveSpeed, turnSpeed);
        driveToWithUpdates(row1Turn, moveSpeed, turnSpeed);
        driveAndIntake(row1End, intakeSpeed, turnSpeed, 8.0);

        telemetry.addData("Phase", "Cycle 2 - shooting row 1");
        telemetry.update();
        driveToWithUpdates(shooting, moveSpeed, turnSpeed);
        shootAllThree();

        // === Cycle 3: intake row 2, return, shoot ===
        telemetry.addData("Phase", "Cycle 3 - intaking row 2");
        telemetry.update();
        driveToWithUpdates(row2Approach, moveSpeed, turnSpeed);
        driveToWithUpdates(row2Turn, moveSpeed, turnSpeed);
        driveAndIntake(row2End, intakeSpeed, turnSpeed, 10.0);

    }
}
