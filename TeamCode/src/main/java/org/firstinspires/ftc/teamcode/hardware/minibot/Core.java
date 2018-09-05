package org.firstinspires.ftc.teamcode.hardware.minibot;

import org.firstinspires.ftc.teamcode.CharlieAutoPrototypeV3;
import org.firstinspires.ftc.teamcode.support.Stuffer;

/**
 * Created by 28761 on 9/4/2018.
 */

public class Core {

    Stuffer stuff;

    private Core() {

    }

    public void set_stuff(Stuffer s) {
        stuff = s;
    }

    public static final Core APPLE_CORE = new Core();
}
