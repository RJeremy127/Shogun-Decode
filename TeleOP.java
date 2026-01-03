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
import org.firstinspires.ftc.teamcode.tools.Intake;
import org.firstinspires.ftc.teamcode.tools.Sorter;
import org.firstinspires.ftc.teamcode.tools.Turret;
import org.firstinspires.ftc.teamcode.tools.tag;
import org.firstinspires.ftc.teamcode.util.Actuation;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="Tele", group="Linear OpMode")
public class TeleOP extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Limelight3A limelight;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() {

        Actuation.setup(hardwareMap, new Pose(0,0,0), telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            LLResult llresult = limelight.getLatestResult();
            if (llresult != null && llresult.isValid()) {
                List<LLResultTypes.FiducialResult> results = llresult.getFiducialResults();
                Pose3D botpose = llresult.getBotpose();
                Point mt1_postion = tag.getPosition(botpose);
                double Tx = llresult.getTx();
                telemetry.addData("Tx: ",  Tx);
                telemetry.addData("Position: ", mt1_postion.toString());
                Turret.track(Tx);
                telemetry.update();
            }
            double axial   =  -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            telemetry.addData("axial: ", axial);
            telemetry.addData("lateral", lateral);
            telemetry.addData("yaw", yaw);

            Actuation.drive(axial,lateral,yaw);

            //String ballColor = Color.getColor();

            if (gamepad1.circle) {
                Intake.intakeBall(0.9, 5);
            }
            if (gamepad1.cross) {
                Intake.intakeBall(-0.9, 5);
            }
            if (gamepad1.right_bumper && !lastRightBumper && !Sorter.isBusy()) {
                Sorter.turn(1);
            }
            if (gamepad1.left_bumper && !lastLeftBumper && !Sorter.isBusy()) {
                Sorter.turn(-1);
            }
            if (gamepad1.right_trigger > 0.9) {
                Turret.turn(-3);
                gamepad1.rumble(100);
            }
            if (gamepad1.left_trigger > 0.9) {
                Turret.turn(3);
                gamepad1.rumble(100);
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Ball Color: ", Arrays.toString(Sorter.getPorts()));
            telemetry.update();
        }
    }
}
