/*
    Drives forward with range sensor pointing behind it. Continues until range sensor detects
    enough space, then it stops.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Nick on 10/6/2017.
 */

@Autonomous(name = "MiniBot: DriveToCryptobox2", group = "MiniBot")
public class DriveToCryptobox2 extends LinearOpMode{
    HardwareMiniBot hw = new HardwareMiniBot();

    int driveDistance = 120; // 120=close, 139=middle, 158=far

    @Override
    public void runOpMode() {
        hw.init(hardwareMap);

        while (opModeIsActive()) {
            telemetry.addData("Distance: ", hw.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            sleep(100);

            hw.leftMotor.setPower(.3);
            hw.rightMotor.setPower(.3);

            if (hw.rangeSensor.getDistance(DistanceUnit.CM) >= driveDistance) {
                hw.leftMotor.setPower(0.0);
                hw.rightMotor.setPower(0.0);
            }
        }
    }
}
