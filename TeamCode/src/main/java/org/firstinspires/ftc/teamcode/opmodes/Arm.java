package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.round;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

import page.j5155.expressway.ftc.motion.FeedforwardFun;
import page.j5155.expressway.ftc.motion.PIDFController;

public class Arm {
    PIDFController controller;
    FeedforwardFun armFF;
    boolean clawToggle = false;
    boolean clawWristToggle;
    public static double p = 0.002, i = 0, d = 0.0001;
    public static double normalF = 0.01;
    public static double zeroF = -0.07;
    public static double fullF = 0.3;
    public static double f = normalF;
    //    public static double f = 0.8;
    private final double ticks_in_degree = round(1993.6 / 360);
    public int armPos = 0;
    boolean poop = true;
    Motor armMotor1;
    Motor armMotor2;
    Slide slides;
    public static PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p,i,d);

    public Arm(HardwareMap hardwareMap) {
        slides = new Slide(hardwareMap);
        armMotor1 = new Motor(hardwareMap.get(DcMotorEx.class, "armMotor1"));
        armMotor2 = new Motor(hardwareMap.get(DcMotorEx.class, "armMotor2"));
        armMotor1.reverse();
        armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armFF = (position, velocity) -> {
            double distanceFromTop = (Math.abs(position - 370) / 100);
//            Log.d("VEGAff", "v " + velocity + " p " + position + " factor " + distanceFromTop);
            if (velocity != null && position > 100) {
                double ff = (position > 370 ? 1.0 : -1.0) * distanceFromTop * Math.abs(velocity / 1000) * -0.05 * (1 + (slides.getPosition()/20.0));
                Log.d("VEGAff", "positive case " + ff);
                return ff;
            }
            else
                return 0;
        };
        controller = new PIDFController(coefficients, armFF);
    }

    public class RunArm implements Action {

        public RunArm () {
            coefficients.setKP(p);
            coefficients.setKI(i);
            coefficients.setKD(d);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armPos = armMotor1.getPosition();
            double power = controller.update(System.nanoTime(), armPos, armMotor1.getVelocity());
//        Log.d("vega", "motor test " + power + " pos " + slideMoztor.getPosition() + " velocity " + slideMotor.getVelocity());
            armMotor1.setPower(power);
            armMotor2.setPower(power);
            return poop;
        }
    }

    public Action runArm() {

        return new RunArm();
    }

    public class SetTarget implements Action {
        int newTarget = 0;
        public SetTarget (int goal) {
            newTarget = goal;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            controller.setTargetPosition(newTarget);
            if (newTarget == 0) {
                coefficients.setKP(p/2);
            } else {
                coefficients.setKP(p);
            }
            return false;
        }
    }
    public Action setTarget(int target){
        return new SetTarget(target);
    }

    public class SetF implements Action {
        int newTarget = 0;
        public SetF (int goal) {
            newTarget = goal;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            controller.setTargetPosition(newTarget);
            if (newTarget == 0) {
                f = -.2;
            } else {
                f = normalF;
            }
            return false;
        }
    }
    public Action setF(int target){
        return new SetF(target);
    }

}

