package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawStuff extends LinearOpMode {
    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");

        waitForStart();
//
//        claw.setPosition(0.25);
//
//        sleep(1000);
//
//        claw.setPosition(0);

        while (opModeIsActive()) {
            if (gamepad1.a) {
                claw.setPosition(0.25);
            } else if (gamepad1.b) {
                claw.setPosition(0);
            }
        }
    }
}
