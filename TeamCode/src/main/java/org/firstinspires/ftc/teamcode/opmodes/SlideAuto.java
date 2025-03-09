package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.opmodes.Arm;

import java.nio.channels.AcceptPendingException;

import page.j5155.expressway.ftc.motion.FeedforwardFun;
import page.j5155.expressway.ftc.motion.PIDFController;




public class SlideAuto {
    public Motor slide;
    PIDFController controller;
    FeedforwardFun armFF;
    public static double p = 0.001, i = 0, d = 0.0001;
    public static double f;
    int newPosition = 0;
    private int targetPosition = 0;

    public static PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p,i,d);
    public SlideAuto (HardwareMap hwMap) {
        slide = new Motor(hwMap.get(DcMotorEx.class, "slides"));
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDFController(coefficients);
    }

    public class RunSlide implements Action {
        public RunSlide() {

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            f = 0.01;
            coefficients.setKP(p);
            coefficients.setKI(i);
            coefficients.setKD(d);
            double power = controller.update(System.nanoTime(), slide.getPosition(), slide.getVelocity());
            slide.setPower(power);

            return true;
        }
    }

    public Action runSlide() {
        return new RunSlide();
    }

    public class prepSlide implements Action {
        public prepSlide() {

        }

        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            slide.runToPosition(390, 1);
            return slide.isBusy();
        }
    }

    public Action prepSlide() {
        return new prepSlide();
    }

    public class clipSlide implements  Action {
        public clipSlide() {

        }

        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            slide.runToPosition(1150, 1);
            return slide.isBusy();
        }
    }

    public Action clipSlide() {
        return new clipSlide();
    }

    public class extendSlide implements Action {

        public extendSlide () {

        }

        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            slide.runToPosition(3000, 1);
            return slide.isBusy();
        }
    }
    public class sample1 implements Action {

        public sample1() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            slide.runToPosition(1200, 1);
            return slide.isBusy();
        }
    }

    public Action sample1 () {
        return new sample1();
    }

    public class sample2 implements Action {

        public sample2() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            slide.runToPosition(100, 1);
            return slide.isBusy();
        }
    }

    public Action sample2 () {
        return new sample2();
    }

    public class RunToPosition implements Action {


        public RunToPosition(int position) {
            newPosition = position;
        }

        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            targetPosition = newPosition;
            controller.setTargetPosition(targetPosition);
            return false;
        }
    }
    public class retractSlide implements Action {
        public retractSlide() {


        }

        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            boolean stop = shouldResetZeroPos();
            slide.runToPositionNoWait(0, 1);
            if (stop) {
                slide.setPower(0);
                slide.reset();
            }

            return !stop;
        }
    }


    private int stuckTime = 0;
    private int lastPos = 0;
    private boolean shouldResetZeroPos() {
        if (Math.abs(slide.getPosition() - lastPos) <= 4) {
            stuckTime++;
            Log.d("Slide", "shouldreset " + stuckTime);
            if (stuckTime >= 30) {
                Log.d("Slide", "reset slide, pos: " + slide.getPosition());
                stuckTime = 0;
                return true;
            }
        }
        lastPos = slide.getPosition();
        return false;
    }

    public Action extendSlide() {

        return new extendSlide();
    }

    public Action retractSlide() {
        return new retractSlide();
    }

    public void setPower (double power) {
        slide.setPower(power);
    }
    public Action runToPosition(int position) {
        return new RunToPosition(position);
    }

    public int getPosition() {
        return slide.getCurrentPosition();
    }
}
