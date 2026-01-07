package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tickle {
    private static Servo tickle;
    private static boolean flicked;
    private static final double fullExtend = 0.75;
    private static final double fullRetract = 0;
    private static final double blockBall = 0.067;

    public static void init(HardwareMap map) {
        tickle = map.get(Servo.class, "tickle");
        retract();
    }

    public static void flick() {
        tickle.setPosition(fullExtend);
        flicked = true;
    }
    public static void retract() {
        tickle.setPosition(fullRetract);
        flicked = false;
    }
    //public static void blockBall() {
     //  tickle.setPosition(blockBall);
      // flicked = true;
    //}
    public static boolean getStatus() {
       return flicked;
    }
}
