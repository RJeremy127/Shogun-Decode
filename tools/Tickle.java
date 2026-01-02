package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Tickle {
    private static Servo right;
    private static Servo left;
    private static boolean flicked;
    private static double fullExtend, fullRetract;

    public static void init(HardwareMap map) {
        right = map.get(Servo.class, "right");
        left = map.get(Servo.class, "left");
        retract();
    }

    public static void flick() {
        right.setPosition(fullExtend);
        left.setPosition(fullExtend);
    }
    public static void retract() {
        right.setPosition(fullRetract);
        left.setPosition(fullRetract);
    }
}
