package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Actuation;

public class gamePad {
    private static Gamepad gamepad1;

    public static void pad1() {
        double axial   =  -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
        Actuation.drive(axial,lateral,yaw);
        if (gamepad1.circle) {
            Intake.intake(0.8);
        }
        if (gamepad1.cross) {
            Intake.intake(-0.8);
        }
    }

    public static void pad2() {

    }
}
