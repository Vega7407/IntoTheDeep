package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

@TeleOp
public class ClawStuff extends LinearOpMode {
    TwoPointServo claw;
    TwoPointServo clawRotate;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = new TwoPointServo(0.25, 0.0, hardwareMap);
        clawRotate = new TwoPointServo(0.25, 0.0, hardwareMap);

        waitForStart();
//
//        claw.setPosition(0.25);
//
//        sleep(1000);
//
//        claw.setPosition(0);

        while (opModeIsActive()) {
            if (gamepad1.a) {
                claw.positionA();
            } else if (gamepad1.b) {
                claw.positionB();
            } else if (gamepad1.x) {
                clawRotate.positionA();
            } else if (gamepad1.y) {
                clawRotate.positionB();
            }
        }
    }
}
