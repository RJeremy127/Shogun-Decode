package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tickle {
    private static Servo right;
    private static Servo left;
    private static boolean flicked;
    private static final double fullExtend = 0.75;
    private static final double fullRetract = 0;
    private static final double blockBall = 0.067;

    public static void init(HardwareMap map) {
        right = map.get(Servo.class, "right");
        left = map.get(Servo.class, "left");
        retract();
    }

    public static void flick() {
        right.setPosition(fullExtend);
        left.setPosition(fullExtend);
        flicked = true;
    }
    public static void retract() {
        right.setPosition(fullRetract);
        left.setPosition(fullRetract);
        flicked = false;
    }
    public static void blockBall() {
       right.setPosition(blockBall);
       left.setPosition(blockBall);
       flicked = true;
    }
    public static boolean getStatus() {
       return flicked;
    }
}
