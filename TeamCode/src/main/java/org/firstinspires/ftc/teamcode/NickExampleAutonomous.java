package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.minibot.Robot;

/**
 * Created by 28761 on 9/4/2018.
 */

@Autonomous(name = "NickExampleAutonomous", group = "MiniBot")
public class NickExampleAutonomous extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();
        robot.init(hardwareMap);

        waitForStart();

        // demonstrate servos
        robot.kicker.kicker_up();
        sleep(1000);
        robot.kicker.kicker_down();
        robot.pusher.pusher_up();
        sleep(1000);
        robot.pusher.pusher_down();
        sleep(2000);

        robot.chassis.drive_distance(.4, 20);
        sleep(500);
        robot.chassis.drive_distance(-.4,20);




    }
}
