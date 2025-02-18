package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp
public class SlideStuff extends LinearOpMode {
    Slide slides;
    DcMotorEx slideMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        slides = new Slide(hardwareMap);
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slides.slide.reset();

        waitForStart();

        while (opModeIsActive()) {
            // these three if statements rotate the slide up when dpad up is pressed,
            // rotate the slide down when dpad down is pressed,
            // and stop the slide when no button is pressed
            if (gamepad1.dpad_up) {
                slideMotor.setPower(0.9);
            } else if (gamepad1.dpad_down) {
                slideMotor.setPower(-.9);
            } else {
                slideMotor.setPower(0);
            }

            // these three if statements turn the motor when dpad up is pressed to extend the slide,
            // turn the motor the other way when dpad down is pressed to retract the slide,
            // and stop the slide when no button is pressed
            if (gamepad1.dpad_right) {
                slides.extendSlide();
            } else if (gamepad1.dpad_left) {
                slides.retractSlide();
            }


            telemetry.addData("current position", slides.slide.getPosition());
            telemetry.update();
        }
    }
}