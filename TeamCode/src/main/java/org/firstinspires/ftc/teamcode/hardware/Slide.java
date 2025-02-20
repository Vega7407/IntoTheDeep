package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;

import org.firstinspires.ftc.teamcode.opmodes.Arm;

import java.nio.channels.AcceptPendingException;


public class Slide {
    public Motor slide;

    public Slide (HardwareMap hwMap) {
        slide = new Motor(hwMap.get(DcMotorEx.class, "slides"));
    }

    public class extendSlide implements Action {

        public extendSlide () {
            slide.runToPositionNoWait(2650, 1);
        }

        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            return slide.isBusy();
        }
    }

    public class retractSlide implements Action {
        public retractSlide() {
            slide.runToPositionNoWait(0, 1);
        }

        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            boolean stop = shouldResetZeroPos();

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

    public int getPosition() {
        return slide.getCurrentPosition();
    }
}
