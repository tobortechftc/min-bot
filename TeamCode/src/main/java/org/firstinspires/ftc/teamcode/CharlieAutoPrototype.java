package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by 28761 on 9/3/2018.
 */

@Autonomous(name = "MiniBot: CharlieAutoTest", group = "MiniBot")
public class CharlieAutoPrototype extends LinearOpMode {

    HardwareMiniBot robot;

    @Override
    public void runOpMode() {
        robot = new HardwareMiniBot();
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();
        waitForStart();

        robot.l_kicker_up();
        sleep(1000);
        robot.r_kicker_up();
        sleep(1000);
        robot.l_kicker_down();
        sleep(1000);
        robot.r_kicker_down();

        try {
            robot.StraightIn(0.5, 20);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        sleep(500);
        try {
            robot.StraightIn(-0.5, 20);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


}
