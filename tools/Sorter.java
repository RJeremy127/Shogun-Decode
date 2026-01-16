package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Sorter {
    private static DcMotor sorter;
    static int targetPosition = 0;
    static double stepTicks = 128.6666666;
    static int currentPort = 0;
    private static int startPos = 1;
    static String[] ports = new String[3];

    // PID gains
    private static double kP = 0.01;
    private static double kI = 0.0;
    private static double kD = 0.0001;
    private static double integralMax = 0.3;
    private static double maxPower = 1.0;
    private static double positionTolerance = 5.0;

    // PID state
    private static double integralSum = 0;
    private static double lastError = 0;
    private static ElapsedTime timer = new ElapsedTime();
    private static double lastTime = 0;

    public static void init(HardwareMap map) {
       sorter = map.get(DcMotor.class, "sorter");
       sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       sorter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       timer.reset();
       lastTime = 0;
       resetPID();
    }

    public static void turn(int turns) {
        targetPosition += (int)(stepTicks * turns);
        // Calculate current port (handle negative modulo correctly)
        currentPort = ((currentPort + turns) % 3 + 3) % 3;
    }

    public static void turnToPort(int port) {
        turn(port - currentPort);
    }

    public static void update() {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        dt = Math.max(dt, 1e-3);

        int currentPos = sorter.getCurrentPosition();
        double error = targetPosition - currentPos;

        if (Math.abs(error) > positionTolerance) {
            // Integral with anti-windup
            integralSum += error * dt;
            integralSum = Math.max(-integralMax, Math.min(integralMax, integralSum));

            // Derivative
            double derivative = (error - lastError) / dt;

            // PID output
            double output = (kP * error) + (kI * integralSum) + (kD * derivative);
            output = Math.max(-maxPower, Math.min(maxPower, output));

            sorter.setPower(output);
            lastError = error;
        } else {
            sorter.setPower(0);
            integralSum = 0;
        }
    }

    public static void resetPID() {
        integralSum = 0;
        lastError = 0;
        lastTime = timer.seconds();
    }

    public static void updatePorts(String ballColor) {
        ports[currentPort] = ballColor;
    }

    public static String[] getPorts() {
        return ports;
    }

    public static int getPosition() {
        return sorter.getCurrentPosition();
    }

    public static int getTargetPosition() {
        return targetPosition;
    }

    public static boolean isBusy() {
        return Math.abs(targetPosition - sorter.getCurrentPosition()) > positionTolerance;
    }

    public static boolean isFull() {
        for (String item : ports) {
            if (item == null) {
                return false;
            }
        }
        return true;
    }

    public static void stop() {
        sorter.setPower(0.0);
        resetPID();
    }

    public static void setStart() {
        // Move to starting position and synchronize internal state
        targetPosition = startPos;
        currentPort = 0;  // Assume start position is port 0
        resetPID();
    }
}
