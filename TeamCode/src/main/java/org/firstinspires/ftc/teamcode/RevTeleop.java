/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="MiniBot: RevTeleop", group="MiniBot")
public class RevTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMiniBot robot           = new HardwareMiniBot();   // Use a Pushbot's hardware
    static double joy_threshold = 0.03;                                                          // could also use HardwarePushbotMatrix class.

    @Override
    public void runOpMode() throws InterruptedException {
        double left=0;
        double right=0;
        double max;
        double prev_right_y=0;
        double prev_left_y=0;
        double speedscale = 0.5;
        int left_event_counter =0;
        int right_event_counter =0;
        int loop_count = 0;
        int delay_scale = 100;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        double power_steps [] = {0.0, 0.5, 0.7, 0.85, 0.95, 1.0};
        int n_steps = 6;
        robot.init(hardwareMap);
        //DcMotor mt = robot.encMotor;
        //robot.encMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        //robot.encMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //int val = robot.encMotor.getCurrentPosition();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        //telemetry.addData("current_encoder =", "%7d", val);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // read IMU angles: heading, roll, pitch

            // read joysticks for tank drive
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            left *= speedscale;
            right *= speedscale;
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            if (gamepad1.y && (speedscale<1.0)) {
                speedscale += 0.05;
            }
            else if (gamepad1.a && (speedscale>0.2)) {
                speedscale -= 0.05;
            }

            if (gamepad1.dpad_right) {
                robot.TurnRightD(0.2, 90.0);
            }


            if (gamepad1.dpad_left) {
                robot.TurnLeftD(0.2, 90.0);
            }
            if (gamepad1.dpad_up) {
                robot.StraightIn(0.6, 40);
            }
            if (gamepad1.dpad_down) {
                robot.StraightIn(-0.6, 40);
            }
            if (gamepad1.left_bumper) {
                robot.goUntilWhite(0.1);
            }
            if (gamepad2.dpad_up) {
                double pos = robot.sv_shoulder.getPosition();
                if (pos < 0.99) {
                    robot.sv_shoulder.setPosition(pos + 0.01);
                }
            }
            if (gamepad2.dpad_down) {
                double pos = robot.sv_shoulder.getPosition();
                if (pos > 0.01) {
                    robot.sv_shoulder.setPosition(pos - 0.01);
                }
            }
            if (gamepad2.dpad_right) {
                double pos = robot.sv_elbow.getPosition();
                if (pos < 0.99) {
                    robot.sv_elbow.setPosition(pos + 0.01);
                }
            }
            if (gamepad2.dpad_left) {
                double pos = robot.sv_elbow.getPosition();
                if (pos > 0.01) {
                    robot.sv_elbow.setPosition(pos - 0.01);
                }
            }
            if (gamepad2.a) {
                robot.sv_elbow.setPosition(0.76);
            }
            if (gamepad2.y) {
                robot.sv_elbow.setPosition(0.1883);
            }
            if (gamepad2.x) {
                robot.sv_shoulder.setPosition(0.4);
            }
            if (gamepad2.b) {
                robot.sv_shoulder.setPosition(0.7094);
            }
            if (gamepad2.left_bumper) {
                robot.sv_shoulder.setPosition(0.54);
            }

            telemetry.addData("left/right motor  =", "%.2f/%.2f", left,right);
            telemetry.addData("speed scale =", "%.2f", speedscale);
            telemetry.addData("tar/curr heading =", "%.2f/%.2f", robot.target_heading, robot.imu_heading());
            telemetry.addData("raw ultrasonic", robot.rangeSensor.rawUltrasonic());
            // telemetry.addData("raw optical", robot.rangeSensor.rawOptical());
            // telemetry.addData("cm optical", "%.2f cm", robot.rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", robot.rangeSensor.getDistance(DistanceUnit.CM));
            //telemetry.addData("current_encoder =", "%d", robot.encMotor.getCurrentPosition());
            // send the info back to driver station using telemetry function.
           // telemetry.addData("LED", ? "On" : "Off");
            telemetry.addData("Clear", robot.colorSensor.alpha());
            telemetry.addData("Red  ", robot.colorSensor.red());
            telemetry.addData("Green", robot.colorSensor.green());
            telemetry.addData("Blue ", robot.colorSensor.blue());
            telemetry.addData("Elbow", "%5.4f", robot.sv_elbow.getPosition());
            telemetry.addData("Shoulder", "%5.4f", robot.sv_shoulder.getPosition());
            // telemetry.addData("current_encoder =", "%d", robot.encMotor.getCurrentPosition());
            // telemetry.addData("current_encoder =", "%7d", encMotor.getCurrentPosition());
            telemetry.update();
            // robot.waitForTick(40);
        }
    }
}
