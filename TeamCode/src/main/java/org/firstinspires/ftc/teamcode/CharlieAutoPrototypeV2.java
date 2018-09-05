package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.minibot.Robot;

/**
 * Created by 28761 on 9/4/2018.
 */

@Autonomous(name = "MiniBot: CharlieAutoTest V2", group = "MiniBot")
public class CharlieAutoPrototypeV2 extends LinearOpMode {

    Robot minibot;

    @Override
    public void runOpMode() throws InterruptedException {
        minibot = new Robot();
        minibot.init(hardwareMap);

        waitForStart();

        //demo on servo controlled component
        minibot.kicker.kicker_up();
        sleep(500);
        minibot.pusher.pusher_up();
        sleep(500);
        minibot.kicker.kicker_down();
        sleep(500);
        minibot.pusher.pusher_down();
        sleep(500);

        // drive a square path
        minibot.chassis.drive_distance(0.5, 20);
        sleep(500);
        minibot.chassis.rotate_right(0.4, 90);
        sleep(500);
        minibot.chassis.drive_distance(0.5, 20);
        sleep(500);
        minibot.chassis.rotate_right(0.4, 90);
        sleep(500);
        minibot.chassis.drive_distance(0.5, 20);
        sleep(500);
        minibot.chassis.rotate_right(0.4, 90);
        sleep(500);
        minibot.chassis.drive_distance(0.5, 20);
        sleep(500);
        minibot.chassis.rotate_right(0.4, 90);
    }
}
