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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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
@TeleOp(name="Tune-Up", group="MiniBot")
public class TuneUp extends LinearOpMode {

    // define robot for the hardware object
    HardwareMiniBot robot = new HardwareMiniBot();
    static double INCREMENT = 0.001;

    @Override
    public void runOpMode() throws InterruptedException {
        double speedscale = 0.5; // determine the maximum speed
        double left;
        double right;
        double max;
        int cur_sv_ix = 0;
        boolean show_all = true;
        boolean tune_up_mode = true;
        boolean braking = true;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.use_imu = true;
        robot.use_minibot = true;
        robot.use_kicker = true;
        robot.use_encoder = true;

        robot.init(hardwareMap);

        Servo[] sv_list = {
                robot.sv_l_kicker,
                robot.sv_r_kicker,
                robot.sv_elbow,
                robot.sv_shoulder
        };
        String [] sv_names = {
                "L-Kicker",
                "R-Kicker",
                "Elbow",
                "Shoulder"
        };

        int num_servos = sv_list.length;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("This is MiniBot.", "Need Tune-up?");    //
        telemetry.update();

        while (cur_sv_ix < num_servos && sv_list[cur_sv_ix] == null) {
            cur_sv_ix++;
        }
        if (cur_sv_ix==num_servos) cur_sv_ix = 0; // no servo available

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (!tune_up_mode) { // tank drive
                left = -gamepad1.left_stick_y;
                right = -gamepad1.right_stick_y;
                // Normalize the values so neither exceed +/- 1.0
                max = Math.max(Math.abs(left), Math.abs(right));
                if (max > 1.0) {
                    left /= max;
                    right /= max;
                }
                left *= speedscale;
                right *= speedscale;
                robot.leftMotor.setPower(left);
                robot.rightMotor.setPower(right);
            } else {
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
                if (gamepad1.left_stick_y < -0.1) {
                    INCREMENT += 0.001;
                    if (INCREMENT > 0.5) INCREMENT = 0.5;
                } else if (gamepad1.left_stick_y > 0.1) {
                    INCREMENT -= 0.001;
                    if (INCREMENT < 0.001) INCREMENT = 0.001;
                }

                if (gamepad1.right_stick_y < -0.1) {
                    speedscale += 0.01;
                    if (speedscale > 1.0) speedscale = 1.0;
                } else if (gamepad1.right_stick_y > 0.1) {
                    speedscale -= 0.01;
                    if (speedscale < 0.1) speedscale = 0.1;
                }
            }
            if (robot.use_minibot) { // Unit Test for minibot
                if (gamepad1.dpad_up) { // forward 24 inches
                    robot.StraightIn(speedscale, 24);
                } else if (gamepad1.dpad_left) { // left turn 90 degree
                    robot.TurnLeftD(speedscale, 90);
                } else if (gamepad1.dpad_right) { // right turn 90 degree
                    robot.TurnRightD(speedscale, 90);
                } else if (gamepad1.dpad_down) { // backward 24 inches
                    robot.StraightIn(-1 * speedscale, 24);
                }
            }
            if (gamepad1.back && gamepad1.b) {
                braking = !braking;
                robot.change_chassis_braking_mode(braking);
                sleep(300);
            } else if (gamepad1.back && gamepad1.a) {
                show_all = !show_all;
                sleep(300);
            } else if (gamepad1.back && gamepad1.y) {
                tune_up_mode = !tune_up_mode;
                sleep(300);
            } else if (gamepad1.back && gamepad1.right_bumper) {
                robot.r_kicker_up();
            } else if (gamepad1.back && (gamepad1.right_trigger>0.1)) {
                robot.r_kicker_down();
            } else if (gamepad1.back && gamepad1.left_bumper) {
                robot.l_kicker_up();
            } else if (gamepad1.back && (gamepad1.left_trigger>0.1)) {
                robot.l_kicker_down();
            } else if (gamepad1.a) {
                if (!tune_up_mode) {
                    // slow down tank speed
                    speedscale -= 0.05;
                    if (speedscale < 0.2) speedscale = 0.2;
                } else if (sv_list[cur_sv_ix] != null) {
                    double pos = sv_list[cur_sv_ix].getPosition();
                    if (pos <= (1 - INCREMENT)) {
                        sv_list[cur_sv_ix].setPosition(pos + INCREMENT);
                    }
                }
                sleep(50);
            } else if (gamepad1.y) {
                if (!tune_up_mode) {
                    // speedup tank drive
                    speedscale += 0.05;
                    if (speedscale > 1.0) speedscale = 1.0;
                } else if (sv_list[cur_sv_ix] != null) {
                    double pos = sv_list[cur_sv_ix].getPosition();
                    if (pos >= INCREMENT) {
                        sv_list[cur_sv_ix].setPosition(pos - INCREMENT);
                    }
                }
                sleep(50);
            }

            if (gamepad1.x) {
                cur_sv_ix--;
                if (cur_sv_ix < 0) cur_sv_ix = num_servos - 1;
                int count = 0;
                while (sv_list[cur_sv_ix] == null && cur_sv_ix >= 0 && count < 20) {
                    cur_sv_ix--;
                    if (cur_sv_ix < 0) cur_sv_ix = num_servos - 1;
                    count++;
                }
                gamepad1.reset();
                sleep(400);
            } else if (gamepad1.b) {
                cur_sv_ix++;
                if (cur_sv_ix >= num_servos) cur_sv_ix = 0;
                int count = 0;
                while (cur_sv_ix < num_servos && sv_list[cur_sv_ix] == null && count < 20) {
                    cur_sv_ix++;
                    if (cur_sv_ix >= num_servos) cur_sv_ix = 0;
                    count++;
                }
                gamepad1.reset();
                sleep(400);
            }
            if (gamepad1.left_bumper) {
                robot.use_imu_correction = !robot.use_imu_correction;
                sleep(400);
            } else if (gamepad1.right_bumper && (robot.correction_ratio<0.99)) {
               robot.correction_ratio += 0.01;
            } else if (gamepad1.right_trigger>0.1 && (robot.correction_ratio>0.5)) {
                robot.correction_ratio -= 0.01;
            }
            if (tune_up_mode) {
                telemetry.addData("0. ", "Tune-Up x/b:sv sel, y/a:+/-(ix=%d)", cur_sv_ix);
            } else {
                telemetry.addData("0. ", "Tank-Drive (back+Y to tune-up)");
            }
            telemetry.addData("0. ", "l/r stick-y: (inc.=%4.3f)/(speed=%2.1f(%s))",
                    INCREMENT,speedscale,(braking?"B":"C"));
            if (show_all) {
                for (int i = 0; i < num_servos; i++) {
                    if (sv_list[i] != null) {
                        telemetry.addData("1.", "%d: %s sv-port %d = %5.4f",
                                i, sv_names[i], sv_list[i].getPortNumber(), sv_list[i].getPosition());
                    }
                }
            } else if (sv_list[cur_sv_ix] != null) {
                telemetry.addData("1. Tune-up servo", " %s (ix=%d) = %5.4f",
                        sv_names[cur_sv_ix], cur_sv_ix, sv_list[cur_sv_ix].getPosition());
            } else {
                telemetry.addLine("1. No active servo to tune-up.");
            }

            if (robot.use_imu) {
                telemetry.addData("2. imu heading = ", "%5.4f (cor.=%s/r=%3.2f)",
                        robot.imu_heading(),
                        (robot.use_imu_correction?"Y":"N"), robot.correction_ratio);
            }
            if (robot.use_minibot) {
                telemetry.addData("2. l/r pwd (enc)=", "%2.1f(%d)/%2.1f(%d)",
                        robot.leftMotor.getPower(),robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getPower(),robot.rightMotor.getCurrentPosition());
            }
            telemetry.update();
        }
    }
}
