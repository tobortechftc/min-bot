package org.firstinspires.ftc.teamcode.hardware.minibot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Kicker {

    Servo kicker;

    final static double KICKER_INIT = 0.558;
    final static double KICKER_UP = 0.46;
    final static double KICKER_DOWN = 0.71;

    public void init(HardwareMap hwMap) {
        kicker = hwMap.servo.get("sv_kicker"); // should be kicker, not sv_kicker

        kicker.setPosition(KICKER_INIT);
    }

    public void kicker_up() {
        kicker.setPosition(.46);
    }
    public void kicker_down() {
        kicker.setPosition(.71);
    }
}
