package org.firstinspires.ftc.teamcode.opmodes;

import static android.os.SystemClock.sleep;
import static java.lang.Math.round;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.hardware.Chassis;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.dairy.pasteurized.PasteurizedGamepad;
import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import page.j5155.expressway.ftc.motion.FeedforwardFun;
import page.j5155.expressway.ftc.motion.PIDFController;

@Config
@TeleOp
public class AllStuffPID extends OpMode {
    SDKGamepad gp1;

    TwoPointServo claw;
    TwoPointServo clawWrist;
    Slide slides;
    Motor slideMotor;
    Motor hangMotor;
    Chassis bobot;
    PIDFController controller;
    boolean clawToggle;
    boolean clawWristToggle;
    public static double p = -0.002, i = 0, d = 0.0001;
    public static double normalF = 0.01;
    public static double zeroF = 0.12;
    public static double fullF = 0.15;
    public static double f = normalF;
//    public static double f = 0.8;
    public static int target;
    boolean hangBoolean = false;
    private final double ticks_in_degree = round(1993.6 / 360);
    public static PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p,i,d);

    @Override
    public void init() {
        gp1 = new SDKGamepad(gamepad1);
        claw = new TwoPointServo(0.18, 0, 1, "claw", hardwareMap);
        clawWrist = new TwoPointServo(0.3, 0.6, 0.9, "clawWrist", hardwareMap);
        claw.positionA();
        clawWrist.positionB();
        slides = new Slide(hardwareMap);
        slideMotor = new Motor(hardwareMap.get(DcMotorEx.class, "slideMotor"));
        hangMotor = new Motor(hardwareMap.get(DcMotorEx.class, "hangMotor"));
        slideMotor.reverse();
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bobot = new Chassis(hardwareMap);
        target = 0;
        FeedforwardFun armFF = (position, velocity) -> {
            double distanceFromTop = (Math.abs(position - 650) / 100);
//            Log.d("VEGAff", "v " + velocity + " p " + position + " factor " + distanceFromTop);
            if (velocity != null && position > 100) {
                double ff = (position > 650 ? 1.0 : -1.0) * distanceFromTop * Math.abs(velocity / 1000) * f;
//                Log.d("VEGAff", "positive case " + ff);
                return ff;
            }
            else
                return 0;
        };
        controller = new PIDFController(coefficients, armFF);

    }
    @Override
    public void loop() {
        coefficients.setKI(i);
        coefficients.setKD(d);
        if (gp1.a().onTrue()) {
            if (clawToggle) {
                claw.positionB();
            } else {
                claw.positionA();
            }
            clawToggle = !clawToggle;
            Log.d("vega", "claw toggle " + clawToggle);
        } else if (gp1.y().onTrue()) {
            clawWrist.positionC();
            clawWristToggle = !clawWristToggle;
        }
        if (gp1.leftStickButton().onTrue()) {
            hangMotor.reverse();
            hangBoolean = !hangBoolean;
            if (hangBoolean){
                hangMotor.setPower(1);
            } else {
                hangMotor.setPower(0);
            }
        }

        if (gp1.x().onTrue()){
            if (clawWristToggle){
                clawWrist.positionA();
            } else {
                clawWrist.positionB();
            }
            clawWristToggle = !clawWristToggle;
            Log.d("vega", "claw wrist toggle " + clawWristToggle);
        } else if (gp1.b().onTrue()) {
            f = normalF;
            target = 680;
            coefficients.setKP(p);
            controller.setTargetPosition(target);
        }

        if (gp1.dpadUp().onTrue()) {
            f = normalF;
            target = (540);
            coefficients.setKP(p);
            controller.setTargetPosition(target);
        } else if (gp1.dpadLeft().onTrue()) {
            f = zeroF;
            target = (1100);
            coefficients.setKP(p);
            controller.setTargetPosition(target);
        } else if (gp1.leftBumper().onTrue()) {
            f = fullF;
            target = (1200);
            coefficients.setKP(p);
            controller.setTargetPosition(target);
        } else if (gp1.dpadDown().onTrue()) {
            f = zeroF;
            target = (0);
            coefficients.setKP(p/1.5);
            controller.setTargetPosition(target);
        } else if (gp1.dpadRight().onTrue()) {
            f = normalF;
            target = (350);
            coefficients.setKP(p);
            controller.setTargetPosition(target);
        } else if (gp1.rightBumper().onTrue()) {
            hangBoolean = !hangBoolean;
            if (hangBoolean){
                hangMotor.setPower(1);
            } else {
                hangMotor.setPower(0);
            }
        }

        int armPos = slideMotor.getPosition();
        double power = controller.update(System.nanoTime(), armPos, slideMotor.getVelocity());
//        Log.d("vega", "motor test " + power + " pos " + slideMoztor.getPosition() + " velocity " + slideMotor.getVelocity());
        slideMotor.setPower(power);



        // these three if z statements turn the motor when dpad up is pressed to extend the slide,
        // turn the motor the other way when dpad down is pressed to retract the slide,
        // and stop the slide when no button is pressed
        // if (gamepad1.dpad_right){
        //     slides.extendSlide();
        // }
        // else if (gamepad1.dpad_left){
        //     slides.retractSlide();
        // }
        
        FtcDashboard.getInstance().getTelemetry().addData("target", target);
        FtcDashboard.getInstance().getTelemetry().addData("position", slideMotor.getPosition());
        FtcDashboard.getInstance().getTelemetry().addData("error", target - slideMotor.getPosition());
        FtcDashboard.getInstance().getTelemetry().update();
        telemetry.addData("a", slideMotor.getPosition());
        telemetry.update();

        double y = gp1.leftStickX().state();
        double x = gp1.leftStickY().state();
        double rx = gp1.rightStickX().state();

        bobot.setMotorPowers(y, x, rx);

        telemetry.addData("rightStick", rx);
    }
}

