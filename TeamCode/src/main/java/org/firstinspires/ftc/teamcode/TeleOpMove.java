package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TeleOpMove extends OpMode {
    public VegaTestHardwareRookie bobot;

    @Override
    public void init() {
        bobot = new VegaTestHardwareRookie(hardwareMap);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad2.right_stick_x;

        bobot.setMotorPowers(y, x, rx);
    }
}
