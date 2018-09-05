package org.firstinspires.ftc.teamcode.hardware.minibot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Pusher {

    Servo pusher;

    final static double PUSHER_INIT = 0.63;
    final static double PUSHER_UP = 0.78;
    final static double PUSHER_DOWN = 0.44;

    public void init(HardwareMap hwMap) {
        pusher = hwMap.servo.get("sv_pusher"); // should be pusher, not sv_pusher

        pusher.setPosition(PUSHER_INIT);
    }

    void pusher_up() {
        pusher.setPosition(PUSHER_UP);
    }

    void pusher_down() {
        pusher.setPosition(PUSHER_DOWN);
    }
}
