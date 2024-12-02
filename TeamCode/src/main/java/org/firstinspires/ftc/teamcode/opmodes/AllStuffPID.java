package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.round;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.hardware.Chassis;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

import page.j5155.expressway.ftc.motion.PIDFController;

@Config
@TeleOp
public class AllStuffPID extends OpMode {
    TwoPointServo claw;
    TwoPointServo clawWrist;
    Slide slides;
    Motor slideMotor;
    Chassis bobot;
    PIDFController controller;
    public static double p = 0.004, i = 0, d = 0.0001;
    public static double f = 0.1;
    int target;
    private final double ticks_in_degree = round(1993.6 / 360);


    @Override
    public void init() {
        claw = new TwoPointServo(0.45, 0.7, "claw", hardwareMap);
        clawWrist = new TwoPointServo(0.35, 0.8, "clawWrist", hardwareMap);
        slides = new Slide(hardwareMap);
        slideMotor = new Motor(hardwareMap.get(DcMotorEx.class, "slideMotor"));
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bobot = new Chassis(hardwareMap);
        target = 0;
        controller = new PIDFController(new PIDFController.PIDCoefficients(p, i, d), 0, 0, f);
    }

    @Override
    public void loop() {
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

        int armPos = slideMotor.getPosition();
        controller.setTargetPosition(target);
        double pid = controller.update(armPos);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;
        slideMotor.setPower(power);

        if (gamepad1.dpad_up) {
            target = (int) (ticks_in_degree * 45);
        } else if (gamepad1.dpad_down) {
            target = 0;
        }

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

