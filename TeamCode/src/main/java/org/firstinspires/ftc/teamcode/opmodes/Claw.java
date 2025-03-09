package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

public class Claw {
    TwoPointServo claw;
    TwoPointServo clawWrist;
    public Claw (HardwareMap hardwareMap) {
        claw = new TwoPointServo(0.445, 0.12, 1, "claw", hardwareMap);
        clawWrist =  new TwoPointServo(0.43, 0.72, 0.16, "clawWrist", hardwareMap);
    }
    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.positionA();
            return false;
        }
    }
    public Action closeClaw() {
        return new CloseClaw();
    }

    public class OpenClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.positionB();
            return false;
        }
    }

    public Action openClaw() {
        return new OpenClaw();
    }

    public class ClawWristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawWrist.positionB();
            return false;
        }
    }

    public Action clawWristUp() {
        return new ClawWristUp();
    }

    public class ClawWristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawWrist.positionA();
            return false;
        }
    }

    public Action clawWristDown() {
        return new ClawWristDown();
    }
}
