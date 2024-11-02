package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp
public class SlideStuff extends LinearOpMode {
    Slide slides = new Slide(hardwareMap);
    DcMotorEx slideMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");

        // these three if statements rotate the slide up when dpad up is pressed,
        // rotate the slide down when dpad down is pressed,
        // and stop the slide when no button is pressed

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                slideMotor.setPower(1);
            } else if (gamepad1.dpad_down) {
                slideMotor.setPower(-.75);
            } else {
                slideMotor.setPower(0);
            }

            // these three if statements turn the motor when dpad up is pressed to extend the slide,
            // turn the motor the other way when dpad down is pressed to retract the slide,
            // and stop the slide when no button is pressed
            if (gamepad1.dpad_right) {
                slides.slide.setPower(1.0);
            } else if (gamepad1.dpad_left) {
                slides.slide.setPower(-1.0);
            } else {
                slides.slide.setPower(0.0);
            }

            telemetry.addData("current position", slides.slide.getPosition());
            telemetry.update();
        }
    }
}