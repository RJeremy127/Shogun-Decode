package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.Arrays;

public class Color {
    private ColorSensor color;
    private int[] rgb;
    private String ballColor;

    public Color(ColorSensor sensor) {this.color = sensor;}

     public void detectColor() {
        rgb = new int[]{color.red(), color.green(), color.blue()};
    }
    public String getColor() {
         //[0]: Red [1]: Green [2]: Blue
        detectColor();
        if (rgb[1] > rgb[0] && rgb[1] > rgb[2] && rgb[0] > 500 && rgb[2] > 500) {
            this.ballColor = "G";
        }
        else if (rgb[2] > rgb[1] && rgb[2] > rgb[0] && rgb[0] > 500 && rgb[1] > 500) {
            this.ballColor = "P";
        }
        else{
            this.ballColor =  null;
        }
        return getBallColor();
    }
    public String getBallColor() {return this.ballColor;}
    public boolean checkMotif(String[] motif, String[] received) {return Arrays.equals(motif,received);}
}
