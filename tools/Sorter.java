package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sorter {
    private static DcMotor sorter;
    static int target = 0;
    static double motorPower = 0.2;
    static int stepTicks = 83;
    static int currentPort = 0;
    static String[] ports = new String[3];

    public static void init(HardwareMap map) {
       sorter = map.get(DcMotor.class, "sorter");
       sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void turn(int turns) {
        target += stepTicks * turns;
        //caculate current port
        currentPort = (currentPort + turns) % 3;
        sorter.setTargetPosition(target);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(motorPower);
    }
    public static void turnToPort(int port) {
        turn(port-currentPort);
    }

    public static void updatePorts(String ballColor) {
        ports[currentPort] = ballColor;
    }

    public static String[] getPorts() {return ports;}

    public static boolean isBusy() {return sorter.isBusy();}
    public static boolean isFull() {
        for (String item : ports) {
            if (item == null) {
                return false;
            }
        }
        return true;
    }
}
