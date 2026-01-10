package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.util.Actuation;

@Autonomous(name="LeftBlue")

public class LeftBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, new Pose(0,0,0), telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        Pose [] p = new Pose[]{new Pose(50, 0, Math.toRadians(90)), new Pose (50, 50, Math.toRadians(180)), new Pose (0, 50, Math.toRadians(270)), new Pose (0, 0, 0)};
        Pose [] poses = new Pose[]{new Pose(0, 10, 0)};
        Route r = new Route(p);
        r.run(0.2, 0.2);

        sleep(10000);
    }
}
