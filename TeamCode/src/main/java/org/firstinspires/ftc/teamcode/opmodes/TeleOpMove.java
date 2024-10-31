package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Motors;

@TeleOp
public class TeleOpMove extends OpMode {
    public Motors bobot;

    @Override
    public void init() {
        bobot = new Motors(hardwareMap);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad2.right_stick_x;

        bobot.setMotorPowers(y, x, rx);
    }
}
