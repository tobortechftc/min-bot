package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.minibot.Robot;
import org.firstinspires.ftc.teamcode.support.YieldHandler;

/**
 * Created by 28761 on 9/4/2018.
 */

public class CharlieAutoPrototypeV3 extends LinearOpMode implements YieldHandler {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot();
        robot.init(hardwareMap);
        robot.core.set_yield_handler(this);
        waitForStart();
    }

    @Override
    public void on_yield() {
        //things to do while waiting
    }

}
