package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Flywheel {
    private static DcMotor wheel;

    public static void init(DcMotor motor) {
        wheel = motor;
    }
}
