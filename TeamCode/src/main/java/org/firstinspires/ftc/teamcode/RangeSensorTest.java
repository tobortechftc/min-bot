package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Nick on 8/25/2017.
 */
@Autonomous(name = "MiniBot: RangeSensorTest", group = "MiniBot")
public class RangeSensorTest extends LinearOpMode {

    ModernRoboticsI2cRangeSensor rangeSensor;
    DcMotor rightMotor;
    DcMotor leftMotor;


    @Override
    public void runOpMode() {

        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
            if (rangeSensor.getDistance(DistanceUnit.CM) <= 20) {
                rightMotor.setPower(0);
                leftMotor.setPower(0);
            } else {
                rightMotor.setPower(.2);
                leftMotor.setPower(.2);
            }
        }
    }
}
