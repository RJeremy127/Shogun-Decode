package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class Color {
    private static ColorSensor color;
    private static ColorSensor color2;
    private static boolean hasSensor2 = false;
    private static int[] rgb;
    private static int[] rgb2;
    private static String ballColor;

    // Purple detection ranges (sensor 1)
    private static final int purpleRedMin = 2000, purpleRedMax = 3000;
    private static final int purpleGreenMin = 2000, purpleGreenMax = 4000;
    private static final int purpleBlueMin = 3600, purpleBlueMax = 6000;

    // Green detection ranges (sensor 1)
    private static final int greenRedMin = 700, greenRedMax = 1900;
    private static final int greenGreenMin = 3800, greenGreenMax = 7000;
    private static final int greenBlueMin = 2000, greenBlueMax = 4000;

    // Sensor 2 thresholds (configurable)
    public static int purple2RedMin = 900, purple2GreenMin = 900, purple2BlueMin = 1700;
    public static int green2RedMin = 250, green2GreenMin = 1200, green2BlueMin = 800;

    public static void init(HardwareMap map) {
        color = map.get(ColorSensor.class, "color");
        try {
            color2 = map.get(ColorSensor.class, "color2");
            hasSensor2 = true;
        } catch (Exception e) {
            color2 = null;
            hasSensor2 = false;
        }
    }

    public static void detectColor() {
        rgb = new int[]{color.red(), color.green(), color.blue()};
        if (hasSensor2) {
            rgb2 = new int[]{color2.red(), color2.green(), color2.blue()};
        }
    }

    public static String getColor() {
        detectColor();
        int r = rgb[0], g = rgb[1], b = rgb[2];

        boolean purple1 = r >= purpleRedMin && r <= purpleRedMax
                && g >= purpleGreenMin && g <= purpleGreenMax
                && b >= purpleBlueMin && b <= purpleBlueMax;

        boolean green1 = r >= greenRedMin && r <= greenRedMax
                && g >= greenGreenMin && g <= greenGreenMax
                && b >= greenBlueMin && b <= greenBlueMax;

        boolean purple2 = false;
        boolean green2 = false;
        if (hasSensor2) {
            int r2 = rgb2[0], g2 = rgb2[1], b2 = rgb2[2];
            purple2 = r2 >= purple2RedMin && g2 >= purple2GreenMin && b2 >= purple2BlueMin;
            green2 = r2 >= green2RedMin && g2 >= green2GreenMin && b2 >= green2BlueMin;
        }

        if (purple1 || purple2) {
            ballColor = "P";
        } else if (green1 || green2) {
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

    public static int[] getRGB2() {
        return rgb2;
    }

    public static boolean hasSensor2() {
        return hasSensor2;
    }

    public static boolean checkMotif(String[] motif, String[] received) {
        return Arrays.equals(motif, received);
    }
}
