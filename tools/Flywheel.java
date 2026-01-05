package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    private static DcMotor wheel;

    public static void init(HardwareMap map) {
        wheel = map.get(DcMotor.class, "flywheel");
    }

    public static void run(double power) {
        wheel.setPower(power);
    }
}
