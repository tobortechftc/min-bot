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
@TeleOp(name="MiniBot: WorkshopTeleop", group="MiniBot")
public class MiniBotWorkshopTeleop extends LinearOpMode {

    // define robot for the hardware object
    HardwareMiniBot robot = new HardwareMiniBot();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double max;
        double speedscale = 0.5; // determine the maximum speed

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            left *= speedscale;
            right *= speedscale;
            robot.leftMotor.setPower(left);
            robot.rightMotor.setPower(right);
            if (gamepad1.y && (speedscale<1.0)) { // speed up
                speedscale += 0.05;
            }
            else if (gamepad1.a && (speedscale>0.2)) { // slow down
                speedscale -= 0.05;
            }

            telemetry.addData("left/right motor  =", "%.2f/%.2f", left,right);
            telemetry.addData("speed scale =", "%.2f", speedscale);
            if (robot.use_imu) {
                telemetry.addData("2. imu heading = ", "%5.4f", robot.imu_heading());
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
