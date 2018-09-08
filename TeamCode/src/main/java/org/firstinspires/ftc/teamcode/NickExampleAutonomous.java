package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.minibot.Robot;
import org.firstinspires.ftc.teamcode.support.OpModeTerminationException;
import org.firstinspires.ftc.teamcode.support.YieldHandler;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by Nick on 9/7/2018.
 */

@Autonomous(name = "NickExampleAutonomous", group = "MiniBot")
public class NickExampleAutonomous extends LinearOpMode implements YieldHandler {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();

        try {
            robot.init(hardwareMap);
        }
        catch (Exception e) {
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
            telemetry.log().add(sw.toString());
            sleep(15000);
            requestOpModeStop();
        }

        waitForStart();

        try {
            robot.kicker.kicker_up();
            sleep(1000);
            robot.kicker.kicker_down();
            robot.pusher.pusher_up();
            sleep(1000);
            robot.pusher.pusher_down();
            sleep(2000);
            robot.chassis.drive_distance(.4, 20);
            sleep(500);
            robot.chassis.drive_distance(-.4, 20);
        }
        catch (OpModeTerminationException e) {
            robot.chassis.stop_chassis();
        }
        catch (Exception e) {
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
            telemetry.log().add(sw.toString());
            robot.chassis.stop_chassis();
        }
    }


    public void on_yield() {
        // Throws an exception if the stop button is pressed.
        if (!opModeIsActive()) {
            throw new OpModeTerminationException();
        }
    }
}
