package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.control.PIDFController;
import org.firstinspires.ftc.teamcode.hardware.Chassis;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;
import org.firstinspires.ftc.teamcode.hardware.Motor;

@TeleOp
public class AllStuff extends LinearOpMode {

    TwoPointServo claw;
    TwoPointServo clawWrist;
    Slide slides;
    Motor slideMotor;
    Chassis bobot;
    PIDFController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = new TwoPointServo(0.45, 0.7, "claw", hardwareMap);
        clawWrist = new TwoPointServo(0.35, 0.8, "clawWrist", hardwareMap);
        slides = new Slide(hardwareMap);
        slideMotor = new Motor(hardwareMap.get(DcMotorEx.class, "slideMotor"));
        bobot = new Chassis(hardwareMap);
        controller = new PIDFController(new PIDFController.PIDCoefficients(1, 0, 0.1));

        waitForStart();

        while (opModeIsActive()) {
            // these two if statements open the servo when A is pressed and close the servo when B is pressed
            if (gamepad1.a) {
                claw.positionA();
            } else if (gamepad1.b) {
                claw.positionB();
            }

            // these two if statements rotate the servo up when Y is pressed and rotate the servo down when X is pressed
            if (gamepad1.y){
                clawWrist.positionA();
            } else if (gamepad1.x) {
                clawWrist.positionB();
            }

            if (gamepad1.dpad_up) {
                controller.setTargetPosition((int) (45 * Motor.getTPD_84()));
            } else if (gamepad1.dpad_down) {
                controller.setTargetPosition(0);
            }

            double power = controller.update(slideMotor.getPosition());
            double error = controller.getTargetPosition() - slideMotor.getPosition();

            telemetry.addData("Target", controller.getTargetPosition());
            telemetry.addData("Position", slideMotor.getPosition());
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.update();

            slideMotor.setPower(power);

//            if (gamepad1.dpad_up) {
//                slideMotor.setPower(.9);
//            } else if (gamepad1.dpad_down) {
//                slideMotor.setPower(-.6);
//            } else if (gamepad1.left_bumper) {
//                slideMotor.setPower(.1);
//            } else if (gamepad1.right_bumper) {
//                slideMotor.setPower(-.1);
//            } else {
//                slideMotor.setPower(0);
//            }
            // these three if z statements turn the motor when dpad up is pressed to extend the slide,
            // turn the motor the other way when dpad down is pressed to retract the slide,
            // and stop the slide when no button is pressed
            if (gamepad1.dpad_right){
                slides.extendSlide();
            }
            else if (gamepad1.dpad_left){
                slides.retractSlide();
            }

            if (gamepad1.back){
                slides.setSlide();
            }

            double y = gamepad1.left_stick_x;
            double x = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            bobot.setMotorPowers(y, x, rx);

        }
    }
}
