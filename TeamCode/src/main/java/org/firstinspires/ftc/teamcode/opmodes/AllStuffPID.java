package org.firstinspires.ftc.teamcode.opmodes;

import static android.os.SystemClock.sleep;
import static java.lang.Math.round;

import android.util.Log;

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
    Chassis bobot;
    PIDFController controller;
    boolean clawToggle;
    boolean clawWristToggle;
    public static double p = -0.002, i = 0, d = 0.0001;
    public static double f = 0.01;
    public static int target;
    private final double ticks_in_degree = round(1993.6 / 360);
    public static PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p,i,d);

    @Override
    public void init() {
        gp1 = new SDKGamepad(gamepad1);
        claw = new TwoPointServo(0.35, 0, "claw", hardwareMap);
        claw.positionB();
        clawWrist = new TwoPointServo(0.40, 0, "clawWrist", hardwareMap);
        clawWrist.positionA();
        slides = new Slide(hardwareMap);
        slideMotor = new Motor(hardwareMap.get(DcMotorEx.class, "slideMotor"));
        slideMotor.reverse();
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bobot = new Chassis(hardwareMap);
        target = 0;

        FeedforwardFun armFF = (position, velocity) -> Math.cos(Math.toRadians(position) / ticks_in_degree) * f;
        controller = new PIDFController(coefficients, armFF);
        clawWrist.positionB();
    }

    @Override
    public void loop() {

        coefficients.setKI(i);
        coefficients.setKD(d);
        // the claw will be toggled between two positions every time a is pressed
        if (gp1.a().onTrue()) {
            if (clawToggle) {
                claw.positionB();
            } else {
                claw.positionA();
            }
            clawToggle = !clawToggle;
            Log.d("vega", "claw toggle " + clawToggle);
        }

        // the claw wrist will be toggled between two positions every time x is pressed
        if (gp1.x().onTrue()){
            if (clawWristToggle){
                clawWrist.positionA();
            } else {
                clawWrist.positionB();
            }
            clawWristToggle = !clawWristToggle;
            Log.d("vega", "claw wrist toggle " + clawWristToggle);

        }

        if (gamepad1.dpad_right) {
            target = (850);
            coefficients.setKP(p/2);
            controller.setTargetPosition(target);
        } else if (gamepad1.dpad_left) {
            target = (400);
            coefficients.setKP(p/2);
            controller.setTargetPosition(target);
        } else if (gamepad1.dpad_down) {
            target = (0);
            coefficients.setKP(p/4);
            controller.setTargetPosition(target);
        }

        int armPos = slideMotor.getPosition();
        double power = controller.update(System.nanoTime(), armPos, slideMotor.getVelocity());
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

        if (gp1.back().onTrue()){
            slides.retractSlide();
        } else if (gamepad1.right_bumper) {
            slides.setSlide();
        }

        FtcDashboard.getInstance().getTelemetry().addData("target", target);
        FtcDashboard.getInstance().getTelemetry().addData("position", slideMotor.getPosition());
        FtcDashboard.getInstance().getTelemetry().addData("error", target - slideMotor.getPosition());
        FtcDashboard.getInstance().getTelemetry().update();

        double y = gp1.leftStickX().state();
        double x = gp1.leftStickY().state();
        double rx = gp1.rightStickX().state();

        bobot.setMotorPowers(y, x, rx);
    }
}

