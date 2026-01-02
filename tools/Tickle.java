package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.Servo;

public class Tickle {
    private static Servo right;
    private static Servo left;
    private static boolean flicked;
    private static double fullExtend, fullRetract;

    public static void init(Servo r, Servo l) {
        right = r;
        left = l;
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
