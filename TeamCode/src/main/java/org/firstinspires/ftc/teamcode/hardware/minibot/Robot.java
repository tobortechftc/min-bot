package org.firstinspires.ftc.teamcode.hardware.minibot;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Chassis chassis;
    public Kicker kicker;
    public Pusher pusher;
    public Core core;

    public Robot() {
        chassis = new Chassis();
        kicker = new Kicker();
        pusher = new Pusher();
        core = new Core();
    }

    public void init(HardwareMap hwMap) {
        chassis.init(hwMap);
        kicker.init(hwMap);
        pusher.init(hwMap);
    }
}
