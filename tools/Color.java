package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class Color {
    private static ColorSensor color;
    private static int[] rgb;
    private static String ballColor;

    public static void init(HardwareMap map) {color = map.get(ColorSensor.class, "color");}

    public static void detectColor() {
        rgb = new int[]{color.red(), color.green(), color.blue()};
    }
    public static String getColor() {
         //[0]: Red [1]: Green [2]: Blue
        detectColor();
        if (rgb[1] > rgb[0] && rgb[1] > rgb[2] && rgb[0] > 100 && rgb[2] > 100) {
            ballColor = "G";
        }
        else if (rgb[2] > rgb[1] && rgb[2] > rgb[0] && rgb[0] > 500 && rgb[1] > 500) {
            ballColor = "P";
        }
        else{
            ballColor =  "No Ball Found";
        }
        return getBallColor();
    }
    public static String getBallColor() {return ballColor;}

    public static int[] getRGB() {
        return rgb;
    }
    public boolean checkMotif(String[] motif, String[] received) {return Arrays.equals(motif,received);}
}
