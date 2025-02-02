package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.round;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

import page.j5155.expressway.ftc.motion.FeedforwardFun;
import page.j5155.expressway.ftc.motion.PIDFController;

public class Arm {
    Motor slideMotor;
    PIDFController controller;
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
    boolean clawToggle = false;
    boolean clawWristToggle;
    public static double p = -0.002, i = 0, d = 0.0001;
    public static double normalF = 0.01;
    public static double zeroF = 0.12;
    public static double fullF = 0.15;
    public static double f = normalF;
    //    public static double f = 0.8;
    public static int target = 530;
    private final double ticks_in_degree = round(1993.6 / 360);
    public int armPos = 0;
    boolean poop = true;
    public static PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p,i,d);

    public Arm(HardwareMap hardwareMap) {
        slideMotor = new Motor(hardwareMap.get(DcMotorEx.class, "slideMotor"));
        slideMotor.reverse();
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDFController(coefficients, armFF);
    }

    public class RunArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            f = normalF;
            coefficients.setKP(p);
            coefficients.setKI(i);
            coefficients.setKD(d);
            controller.setTargetPosition(target);
            armPos = slideMotor.getPosition();
            double power = controller.update(System.nanoTime(), armPos, slideMotor.getVelocity());
            slideMotor.setPower(power);
            return poop;
        }
    }

    public Action runArm() {
        return new RunArm();
    }

    public class SetTarget implements Action {
        public SetTarget (int goal) {
            target = goal;
            controller.setTargetPosition(target);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return false;
        }
    }
    public Action setTarget(int target){
        return new SetTarget(target);
    }


}

