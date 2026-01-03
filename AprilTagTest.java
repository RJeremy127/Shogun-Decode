package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.datatypes.Point;
import org.firstinspires.ftc.teamcode.tools.tag;

import java.util.Arrays;
import java.util.List;

@Autonomous
public class AprilTagTest extends OpMode{
    private Limelight3A limelight;

    private Servo test;

    private DcMotor turn;
    private double mountAngle = 0.0;

    //height of the Apriltag (in cm)
    private double TagHeight = 74.95;
    //height of the Limelight (in cm)
    private double LmHeight = 30.5;

    private String[] motif = {};

    private int pipeline = 7;

    private double trackSpeed = 0.5;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pipeline = 7;
        //pipeline 7 is ID21,22,23  
        //pipeline 8 is ID20(BLUE)
        //pipeline 9 is ID24(RED)
        limelight.pipelineSwitch(pipeline);
        turn = hardwareMap.dcMotor.get("turnTable");
    }

    @Override
    public void start(){
        telemetry.addLine("initialized");
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()) {
            List<LLResultTypes.FiducialResult> results = llresult.getFiducialResults();
            Pose3D botpose = llresult.getBotpose();
            Pose3D botpose_mt2 = llresult.getBotpose_MT2();

            Point mt1_postion = tag.getPosition(botpose);
            Point mt2_position = tag.getPosition(botpose_mt2);

            int tagID = tag.getTagID(results);
            motif = tag.getMotif(tagID);
            double Tx = llresult.getTx();
            double Ty = llresult.getTy();
            double Ta = llresult.getTa();
            double distance = tag.getDistance(mountAngle, Ty, LmHeight, TagHeight);

            DistanceUnit unit = botpose.getPosition().unit;

            tag.trackTag(turn, trackSpeed, Tx);

            telemetry.addData("April Tag ", tagID);
            telemetry.addData("Target x", Tx);
            telemetry.addData("Target y", Ty);
            telemetry.addData("Target Area", Ta);
            //telemetry.addData("Yaw", botpose_mt2.getOrientation());
            telemetry.addData("Distance", distance);
            telemetry.addData("MT1 Coordinates", mt1_postion.toString());
            telemetry.addData("Motif colors", Arrays.toString(motif));
            telemetry.addData("unit", unit.toString());
        }

        else {
            telemetry.addLine("No AprilTag detected");
            telemetry.addData("Pipeline", pipeline);
            turn.setPower(0);
        }

    }
}
