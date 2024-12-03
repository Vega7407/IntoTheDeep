package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.round;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.hardware.Chassis;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

import page.j5155.expressway.ftc.motion.FeedforwardFun;
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
    public static double p = -0.002, i = 0, d = 0.01;
    public static double f = 0.01;
    public static int target;
    private final double ticks_in_degree = round(1993.6 / 360);
    public static PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p,i,d);

    public class ArmFeedForward implements FeedforwardFun {

        @Override
        public Double apply(Double aDouble, Double aDouble2) {
            return Math.cos(Math.toRadians(aDouble) / ticks_in_degree ) * f;
        }

        @NonNull
        @Override
        public Double apply(double v, @Nullable Double aDouble) {
            return apply(Double.valueOf(v), aDouble);
        }
    }

    @Override
    public void init() {
        claw = new TwoPointServo(0.45, 0.7, "claw", hardwareMap);
        clawWrist = new TwoPointServo(0.35, 0.8, "clawWrist", hardwareMap);
        slides = new Slide(hardwareMap);
        slideMotor = new Motor(hardwareMap.get(DcMotorEx.class, "slideMotor"));
        slideMotor.reverse();
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bobot = new Chassis(hardwareMap);
        target = 0;
        controller = new PIDFController(coefficients, new ArmFeedForward());
        // (Double position, Double velocity) -> Math.cos(Math.toRadians(position) / ticks_in_degree) * f
    }

    @Override
    public void loop() {
        coefficients.setKP(p);
        coefficients.setKI(i);
        coefficients.setKD(d);
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
            target = (550);
            controller.setTargetPosition(target);
        } else if (gamepad1.dpad_down) {
            target = 0;
            slideMotor.setPower(0.1);
            controller.setTargetPosition(target);

        }

        int armPos = slideMotor.getPosition();
        double power = controller.update(armPos);
        slideMotor.setPower(power);



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

        FtcDashboard.getInstance().getTelemetry().addData("target", target);
        FtcDashboard.getInstance().getTelemetry().addData("position", slideMotor.getPosition());
        FtcDashboard.getInstance().getTelemetry().addData("error", target - slideMotor.getPosition());
        FtcDashboard.getInstance().getTelemetry().update();

        double y = gamepad1.left_stick_x;
        double x = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        bobot.setMotorPowers(y, x, rx);
    }
}

