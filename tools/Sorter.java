package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Sorter {
    private static DcMotor motor;
    static int target = 0;
    static double motorPower = 0.2;
    static int stepTicks = 83;
    static int currentPort = 0;
    static String[] ports = new String[3];

    public static void init(DcMotor mt) {
        motor = mt;
    }

    public static void turn(int turns) {
        target += stepTicks * turns;
        //caculate current port
        currentPort = (currentPort + turns) % 3;
        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(motorPower);
    }
    public static void turnToPort(int port) {
        turn(port-currentPort);
    }

    public static void updatePorts(String ballColor) {
        ports[currentPort] = ballColor;
    }
    public static boolean isFull() {
        for (String item : ports) {
            if (item == null) {
                return false;
            }
        }
        return true;
    }
}
