/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.database.AbstractCursor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.datatypes.Point;
import org.firstinspires.ftc.teamcode.datatypes.Pose;
import org.firstinspires.ftc.teamcode.tools.Color;
import org.firstinspires.ftc.teamcode.tools.Flywheel;
import org.firstinspires.ftc.teamcode.tools.Intake;
import org.firstinspires.ftc.teamcode.tools.JohnLimeLight;
import org.firstinspires.ftc.teamcode.tools.Sorter;
import org.firstinspires.ftc.teamcode.tools.Tickle;
import org.firstinspires.ftc.teamcode.tools.Turret;
import org.firstinspires.ftc.teamcode.util.Actuation;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="Tele", group="Linear OpMode")
public class TeleOP extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Limelight3A limelight;
    private boolean lastRight = false;
    private boolean lastLeft = false;
    private boolean isTrack = false;
    private boolean isFlicked = false;
    private boolean isSpinningUp = false;
    private double targetFlywheelVelocity = 0;
    private JohnLimeLight.Alliance alliance = JohnLimeLight.Alliance.BLUE;

    @Override
    public void runOpMode() {
        Actuation.setup(hardwareMap, new Pose(0, 0, 0), telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        JohnLimeLight.switchToObelisk(limelight);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        limelight.start();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            isTrack = false;
            isFlicked = Tickle.getStatus();
            LLResult llresult = limelight.getLatestResult();
            //if (!Intake.isBusy()) {Tickle.blockBall();}

            // Update flywheel PID controller
            Flywheel.update();

            if (llresult != null && llresult.isValid() && isTrack) {
                List<LLResultTypes.FiducialResult> results = llresult.getFiducialResults();
                Pose3D botpose = llresult.getBotpose();
                Point mt1_postion = JohnLimeLight.getPosition(botpose);
                double Tx = llresult.getTx();
                telemetry.addData("Tx: ", Tx);
                telemetry.addData("Position: ", mt1_postion.toString());
                Turret.track(Tx);
            }
            //String ballColor = Color.getColor();
            pad1();
            pad2();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Is tracking: ", isTrack);
            telemetry.addData("Color: ", Color.getColor());
            telemetry.addData("Flywheel Target: ", String.format("%.0f", targetFlywheelVelocity));
            telemetry.addData("Flywheel Ready: ", isSpinningUp && Flywheel.isAtSpeed(50));
            //telemetry.addData("Ball Color: ", Arrays.toString(Sorter.getPorts()));
            telemetry.update();
        }
    }
    // gamepad 1: drive + intake
    // gamepad 1: turret + shoot
    public void pad1() {
        double axial   =  -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
        Actuation.drive(axial,lateral,yaw);
        if (gamepad1.circle && !isFlicked) {
            Tickle.retract();
            Intake.intake(0.8);
        }
        if (gamepad1.cross && !isFlicked) {
            Tickle.retract();
            Intake.intake(-0.8);
        }
        if (gamepad1.right_bumper && !Sorter.isBusy() && !isFlicked) {
            Sorter.turn(1);
        }
        if (gamepad1.left_bumper && !Sorter.isBusy() && !isFlicked) {
            Sorter.turn(-1);
        }

    }
    public void pad2() {
        // Turret control
        if (gamepad2.square) {
            isTrack = true;
        }
        else {
            Turret.turn((int)(gamepad2.left_stick_x * 5));
        }

        // Flywheel shooting with PID velocity control
        if (gamepad2.right_trigger > 0.2) {
            // Hard-coded target velocity (adjust this value for your robot)
            targetFlywheelVelocity = 1500;  // ticks/sec
            Flywheel.setTargetVelocity(targetFlywheelVelocity);
            isSpinningUp = true;

            // Auto-flick when flywheel reaches target speed (tolerance: 50 ticks/sec)
            if (Flywheel.isAtSpeed(50)) {
                Tickle.flick();
            }
        }
        // Stop flywheel when trigger released
        else {
            targetFlywheelVelocity = 0;
            Flywheel.setTargetVelocity(0);
            isSpinningUp = false;
        }
    }
}

