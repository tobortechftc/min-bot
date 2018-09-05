package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.minibot.Core;
import org.firstinspires.ftc.teamcode.support.Stuffer;

/**
 * Created by 28761 on 9/4/2018.
 */

public class CharlieAutoPrototypeV3 extends LinearOpMode implements Stuffer {

    @Override
    public void runOpMode() throws InterruptedException {
        Core.APPLE_CORE.set_stuff(this);
    }

    @Override
    public void do_stuff() {

    }

}
