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
    // Purple detection ranges
    private static final int purpleRedMin = 2000, purpleRedMax = 3000;
    private static final int purpleGreenMin = 2000, purpleGreenMax = 4000;
    private static final int purpleBlueMin = 3600, purpleBlueMax = 6000;

    // Green detection ranges
    private static final int greenRedMin = 700, greenRedMax = 1900;
    private static final int greenGreenMin = 3800, greenGreenMax = 7000;
    private static final int greenBlueMin = 2000, greenBlueMax = 4000;

    public static String getColor() {
        detectColor();
        int r = rgb[0], g = rgb[1], b = rgb[2];

        if (r >= purpleRedMin && r <= purpleRedMax
                && g >= purpleGreenMin && g <= purpleGreenMax
                && b >= purpleBlueMin && b <= purpleBlueMax) {
            ballColor = "P";
        } else if (r >= greenRedMin && r <= greenRedMax
                && g >= greenGreenMin && g <= greenGreenMax
                && b >= greenBlueMin && b <= greenBlueMax) {
            ballColor = "G";
        } else {
            ballColor = null;
        }
        return getBallColor();
    }
    public static String getBallColor() {return ballColor;}

    public static int[] getRGB() {
        return rgb;
    }
    public boolean checkMotif(String[] motif, String[] received) {return Arrays.equals(motif,received);}
}
