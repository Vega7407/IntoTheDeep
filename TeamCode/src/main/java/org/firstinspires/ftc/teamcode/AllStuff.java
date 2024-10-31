package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class AllStuff extends LinearOpMode {
    Servo claw;
    Servo clawWrist;
    DcMotorEx slides;
    DcMotorEx slideMotor;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // these two if statements open the servo when A is pressed and close the servo when B is pressed
            if (gamepad1.a) {
                claw.setPosition(0.3);
            } else if (gamepad1.b) {
                claw.setPosition(0);
            }

            // these two if statements rotate the servo up when Y is pressed and rotate the servo down when X is pressed
            if (gamepad1.y){
                clawWrist.setPosition(0.25);
            } else if (gamepad1.x) {
                clawWrist.setPosition(0);
            }

            // these three if statements rotate the slide up when dpad up is pressed,
            // rotate the slide down when dpad down is pressed,
            // and stop the slide when no button is pressed
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
            if (gamepad1.dpad_right){
                slides.setPower(.75);
            }
            else if (gamepad1.dpad_left){
                slides.setPower(-.65);
            }
            else {
                slides.setPower(0);
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            frontLeft.setPower(y + x + rx);
            backLeft.setPower(y - x + rx);
            frontRight.setPower(y - x - rx);
            backRight.setPower(y + x - rx);
        }
    }
}
