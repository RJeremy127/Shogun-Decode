package org.firstinspires.ftc.teamcode.util;

import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;

public class Constants {
    public static int BACK_RIGHT = 0; // Control Hub 0 // right
    public static int BACK_LEFT = 1; // Expansion Hub 0 // left

    public static int FRONT_RIGHT = 2; // Control Hub 1 // back
    public static int FRONT_LEFT = 3; // Expansion Hub 1
    //public static String[] MOTOR_MAP = {"backRight", "backLeft", "frontRight", "frontLeft"};
    Map<String, Integer> map = new HashMap<String, Integer>()
    {{put("backRight", 0);}};
}
