package org.firstinspires.ftc.teamcode.opmodes;

import static android.os.SystemClock.sleep;
import static java.lang.Math.round;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.hardware.Chassis;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.dairy.pasteurized.PasteurizedGamepad;
import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import page.j5155.expressway.ftc.motion.FeedforwardFun;
import page.j5155.expressway.ftc.motion.PIDFController;

@Config
@TeleOp
public class AllStuffPIDNewBot extends OpMode {
    SDKGamepad gp1;

    TwoPointServo claw;
    TwoPointServo clawWrist;
    TwoPointServo sweep;
    Slide slides;
    Motor armMotor1;
    Motor armMotor2;
    Chassis bobot;
    PIDFController controller;
    boolean clawToggle;
    boolean clawWristToggle;
    boolean sweepToggle;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    public static double p = 0.0027, i = 0, d = 0.0002;
    public static double normalF = 0.01;
    public static double zeroF = -0.2;
    public static double fullF = 0.3;
    public static double f = normalF;
    //    public static double f = 0.8;
    public static int target;
    boolean hangBoolean = false;
    private final double ticks_in_degree = round(1993.6 / 360);
    public static PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p,i,d);

    @Override
    public void init() {
        gp1 = new SDKGamepad(gamepad1);
        claw = new TwoPointServo(0.38, 0.15, 1, "claw", hardwareMap);
        clawWrist =  new TwoPointServo(0.43, 0.72, 0.16, "clawWrist", hardwareMap);
        sweep = new TwoPointServo(0.31, 0.34, 0.7, "sweep", hardwareMap);
        slides = new Slide(hardwareMap);
        armMotor1 = new Motor(hardwareMap.get(DcMotorEx.class, "armMotor1"));
        armMotor2 = new Motor(hardwareMap.get(DcMotorEx.class, "armMotor2"));
        armMotor1.reverse();
        armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bobot = new Chassis(hardwareMap);
        target = 0;
        FeedforwardFun armFF = (position, velocity) -> {
            double distanceFromTop = (Math.abs(position - 370) / 100);
//            Log.d("VEGAff", "v " + velocity + " p " + position + " factor " + distanceFromTop);
            if (velocity != null && position > 100) {
                double ff = (position > 370 ? 1.0 : -1.0) * distanceFromTop * Math.abs(velocity / 1000) * f * (1 + (slides.getPosition()/20.0));
                Log.d("VEGAff", "positive case " + ff);
                return ff;
            }
            else
                return 0;
        };
        controller = new PIDFController(coefficients, armFF);

    }
    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

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
        }

        if (gp1.rightStickButton().onTrue()) {
            if (sweepToggle) {
                sweep.positionB();
            } else {
                sweep.positionC();
            }
            sweepToggle = !sweepToggle;
        }

//
        if (gp1.x().onTrue()) {
            if (clawWristToggle) {
                clawWrist.positionA();
            } else {
                clawWrist.positionB();
            }
            clawWristToggle = !clawWristToggle;
            Log.d("vega", "claw wrist toggle " + clawWristToggle);
        }

        if (gp1.b().onTrue()) {
            f = 0.01;
            target = (360);
            coefficients.setKP(p);
            clawWrist.positionB();
            controller.setTargetPosition(target);
            runningActions.add(slides.runToPosition(500));
        } else if (gp1.dpadUp().onTrue()) {
            f = 0.01;
            target = (360);
            coefficients.setKP(p);
            controller.setTargetPosition(target);
        } else if (gp1.dpadLeft().onTrue()) {
            runningActions.add(slides.runToPosition(1100));
        } else if (gp1.leftBumper().onTrue()) {
            clawWrist.positionB();
            runningActions.add(slides.retractSlide());
//            slides.extendSlide();
        } else if (gp1.rightBumper().onTrue()) {
            slides.setPower(1);
        } else if (gp1.y().onTrue() && Math.abs(armMotor1.getPosition()) > 20) {
            runningActions.add(slides.extendSlide());
        } else if (gp1.rightBumper().onFalse() && !gp1.back().onTrue()) {
            slides.setPower(0);
        } else if (gp1.back().onTrue()) {
            slides.setPower(-1);
        } else if (gp1.back().onFalse() && !gp1.rightBumper().onTrue()) {
            slides.setPower(0);
        }

        if (gp1.rightStickButton().onTrue()) {
            target = (0);
            coefficients.setKP(p*8);
            controller.setTargetPosition(target);
        }

        if (gp1.dpadDown().onTrue() && Math.abs(slides.getPosition()) < 20) {
            f = zeroF;
            target = (0);
            coefficients.setKP(p / 1.5);
            controller.setTargetPosition(target);
            clawWrist.positionB();
        } else if (gp1.dpadRight().onTrue()) {
            f = -0.05;
            target = (210);
            coefficients.setKP(p);
            runningActions.add(slides.retractSlide());
            controller.setTargetPosition(target);
            clawWrist.setPosition(0.34);
        } else if (Math.abs(armMotor1.getPosition()) < 20 && slides.getPosition() > 1900) {
            if (gp1.leftBumper().onTrue()) {

            } else if (gp1.leftBumper().onFalse()){
                slides.setPower(0);
            }
        }


        int armPos = armMotor1.getPosition();
        double power = controller.update(System.nanoTime(), armPos, armMotor1.getVelocity());
        Log.d("vega", "motor test " + power + " pos " + slides.getPosition());
        if (gp1.dpadDown().onTrue()) {

        } else if (gp1.dpadLeft().onTrue()) {
            power = power * (1 + slides.getPosition()/1700.0);
        } else {
            power = power * (1 + slides.getPosition()/1500.0);
        }
        armMotor1.setPower(power);
        armMotor2.setPower(power);



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
        FtcDashboard.getInstance().getTelemetry().addData("position", armMotor1.getPosition());
        FtcDashboard.getInstance().getTelemetry().addData("error", target - armMotor1.getPosition());
        FtcDashboard.getInstance().getTelemetry().update();

        telemetry.addLine("VEGA 7407");
        telemetry.addData("armMotor", armMotor1.getPosition());
        telemetry.addData("armPower", power);
        telemetry.addData("slidesPos", slides.getPosition());
        telemetry.addLine("");
        telemetry.addData("claw", claw.getPosition());
        telemetry.addData("clawWrist", clawWrist.getPosition());
        telemetry.addData("sweeper", sweep.getPosition());
        telemetry.addLine("");
        telemetry.addData("frontLeft", bobot.frontLeft.getPower());
        telemetry.addData("backLeft", bobot.backLeft.getPower());
        telemetry.addData("backRight", bobot.backRight.getPower());
        telemetry.addData("frontRight", bobot.frontRight.getPower());
        telemetry.addLine("");
        telemetry.addData("frontLeft", bobot.frontLeft.getDirection());
        telemetry.addData("backLeft", bobot.backLeft.getCurrentPosition());
        telemetry.addData("backRight", bobot.backRight.getCurrentPosition());
        telemetry.addData("frontRight", bobot.frontRight.getPosition());
//        telemetry.update();

        double y = gp1.leftStickY().state();
        double x = gp1.leftStickX().state();
        double rx = gp1.rightStickX().state();
        if (Math.abs(armMotor1.getPosition() - 150) < 30) {
            bobot.setMotorPowersSlow(y, x, rx);
        } else {
            bobot.setMotorPowers(y, x, rx);
        }

        telemetry.addData("rightStick", rx);
    }
}
