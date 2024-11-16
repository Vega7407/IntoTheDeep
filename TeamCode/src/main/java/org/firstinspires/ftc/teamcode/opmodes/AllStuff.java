package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Motor.CPR_84;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
//    Gamepad lastGamepad1;
//    double target;
//    double position;
//    double error;
//    double power;
//    double kP = 1;
//    final int tolerance = 700;
//    boolean doPID = true;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = new TwoPointServo(0.45, 0.7, "claw", hardwareMap);
        clawWrist = new TwoPointServo(0.35, 0.8, "clawWrist", hardwareMap);
        slides = new Slide(hardwareMap);
        slideMotor = new Motor(hardwareMap.get(DcMotorEx.class, "slideMotor"));
//        slideMotor.getInternal().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bobot = new Chassis(hardwareMap);
//        lastGamepad1 = new Gamepad();
//        slideMotor.reset();




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
                slideMotor.setPower(.9);
            } else if (gamepad1.dpad_down) {
                slideMotor.setPower(-.6);
            } else if (gamepad1.right_bumper) {
                slideMotor.setPower(.1);
            } else {
                slideMotor.setPower(0);
            }
//            if (gamepad1.dpad_up) {
//                target = CPR_84/2.45;
//                kP = 1;
//            } else if (gamepad1.dpad_down) {
//                target = 1;
//                kP = 0.05 ;
//            } else if (gamepad1.right_bumper) {
//                doPID = !doPID;
//            }
//            position = -slideMotor.getPosition();
//            error = target - position;
//
//            power = error * kP;
//
//            if (doPID && Math.abs(error) > tolerance) {
//                slideMotor.getInternal().setPower(power);
//            } else {
//                power = 0;
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

            if (gamepad1.left_bumper){
                slides.setSlide();
            }

            double y = gamepad1.left_stick_x;
            double x = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            bobot.setMotorPowers(y, x, rx);

//            telemetry.addData("Target", target);
//            telemetry.addData("Prop", error/target);
//            telemetry.addData("Error", error);
//            telemetry.addData("Pos", position);
//            telemetry.addData("Pow", power);
            telemetry.addData("claw", claw.claw.getPosition());
            telemetry.addData("wrist", clawWrist.claw.getPosition());
            telemetry.update();
        }
    }
}
