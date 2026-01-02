package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.datatypes.Point;

import java.util.List;

public class JohnLimeLight{
    private Limelight3A limelight;
    private enum alliance {BLUE, RED};
    private LLResult llresult;
    private Pose3D botpose;
    private Point position;
    private int tagID;
    private List<LLResultTypes.FiducialResult> results;
    private String[] motif;
    private double mountAngle, tagHeight,lmHeight,Tx,Ty,Ta;


    public JohnLimeLight(Limelight3A ll, double MA, double TH, double LH) {
        this.limelight = ll;
        this.mountAngle = MA;
        this.tagHeight = TH;
        this.lmHeight = LH;
    }
    public void update() {
        this.llresult = limelight.getLatestResult();
        this.botpose = llresult.getBotpose();
        this.results = llresult.getFiducialResults();
        this.Tx = llresult.getTx();
        this.Ty = llresult.getTy();
        this.Ta = llresult.getTa();
    }
    public int getTagID() {
        for (LLResultTypes.FiducialResult fiducial : this.results) {
            this.tagID = fiducial.getFiducialId();
        }
       return tagID;
    }
    public String[] getMotif() {
        switch (tagID) {
            case 21 -> motif = new String[]{"G", "P", "P"};
            case 22 -> motif = new String[]{"P", "G", "P"};
            case 23 -> motif = new String[]{"P", "P", "G"};
        }
        return motif;
    }

    public double getDistance() {
        double angToGoalDeg = this.Ta + this.mountAngle;
        double angleRadians = angToGoalDeg * (Math.PI / 180.0);
        return (lmHeight - tagHeight) / Math.tan(angleRadians);
    }

    public Point getPosition() {
        if (this.botpose != null) {
            double x = botpose.getPosition().x;
            double y = botpose.getPosition().y;
            x *= 39.3701;
            y *= 39.3701;
            return new Point(x,y);
        }
        else {return null;}
    }

    public boolean inZone(Point point) {
        double x = point.getX();
        double y = point.getY();
        if (((y < 72) && (y > Math.abs(x))) || ((y > 72) && (y < -Math.abs(x)-48))) {
            return true;
        }
        else {return false;}
    }

    public void switchToGoal(tag.Alliance alliance) {
        //pipeline 8 = blue
        //pipleline 9 = red
        limelight.pipelineSwitch(alliance == tag.Alliance.BLUE ? 8:9);
    }
    public void switchObelisk() {
        limelight.pipelineSwitch(7);
    }
    public boolean isAligned(double tolerance) {
        return Math.abs(this.Tx) <= tolerance;
    }
}
