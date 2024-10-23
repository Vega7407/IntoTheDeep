package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SlideStuff extends OpMode {
    DcMotorEx slides;

    @Override
    // hardwareMap must be inside init or loop as anything outside is not a part of OpMode
    public void init() {
        slides = hardwareMap.get(DcMotorEx.class, "slides");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up){
            slides.setPower(.65);
        }
        else if (gamepad1.dpad_down){
            slides.setPower(-.65);
        }
        else {
            slides.setPower(0);
        }
    }
}
