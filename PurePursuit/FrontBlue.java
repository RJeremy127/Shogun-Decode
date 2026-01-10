package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.tools.Flywheel;
import org.firstinspires.ftc.teamcode.tools.Sorter;
import org.firstinspires.ftc.teamcode.tools.Tickle;
import org.firstinspires.ftc.teamcode.tools.Turret;
import org.firstinspires.ftc.teamcode.util.Actuation;

@Autonomous(name="FrontBlue")
public class FrontBlue extends LinearOpMode {
    public void runOpMode() {
        Actuation.setup(hardwareMap, new Pose(0,0,0), telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // turret/movement align
        Pose [] p = new Pose[]{new Pose(-40, 40, Math.toRadians(0))};
        Route r = new Route(p);
        r.run(.2, .2);
        //Turret.track();
        for (int i = 0; i<2;i++) {
            launch();
        }
        Pose [] a = new Pose[]{new Pose(0, -10, Math.toRadians(0))};
        Route c = new Route(a);
        c.run(.2, .2);
    }
    public void launch() {
        Flywheel.setTargetVelocity(1550);
        sleep(1000);
        Tickle.flick();
        Tickle.blockBall();
        Sorter.turn(1);
    }
}
