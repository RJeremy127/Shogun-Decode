package org.firstinspires.ftc.teamcode.PurePursuit;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.tools.Flywheel;
import org.firstinspires.ftc.teamcode.tools.Sorter;
import org.firstinspires.ftc.teamcode.tools.Tickle;
import org.firstinspires.ftc.teamcode.tools.Turret;
import org.firstinspires.ftc.teamcode.util.Actuation;

import java.util.List;

//@Autonomous(name="BackRed")
public class BackRed extends LinearOpMode {
    private Limelight3A lim;
    double Tx, Ty;
    public void runOpMode() {
        Actuation.setup(hardwareMap, new Pose(0,0,0), telemetry);
        lim = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        lim.start();
        waitForStart();
        //8 is blue
        //9 is red
        lim.pipelineSwitch(9);
        LLResult llresult = lim.getLatestResult();

        while (!(llresult != null && llresult.isValid())) {
            Turret.turn(20);
        }

        do {
            List<LLResultTypes.FiducialResult> results = llresult.getFiducialResults();
            Tx = llresult.getTx();
            Ty = llresult.getTy();
        } while(Turret.autoTrack(Tx, Ty));
        launch();
        Pose [] p = new Pose[]{new Pose(0, 20, Math.toRadians(0))};
        Route r = new Route(p);
        r.run(.5, .2);
    }
    public void launch() {
        Flywheel.setTargetVelocity(Flywheel.calculateTargetVelocity(Ty));
        sleep(1000);
        Tickle.flick();
        Tickle.blockBall();
    }
}
