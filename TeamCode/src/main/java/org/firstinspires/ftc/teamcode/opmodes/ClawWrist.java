package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

public class ClawWrist {
    TwoPointServo clawWrist;
    public ClawWrist(HardwareMap hardwareMap) {
        clawWrist =  new TwoPointServo(0.43, 0.72, 0.16, "clawWrist", hardwareMap);
        clawWrist.positionA();
    }

    public class ClawWristUp implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawWrist.positionA();
            return false;
        }
    }

    public Action clawWristUp() {
        return new ClawWristUp();
    }

    public class ClawWristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawWrist.positionB();
            return false;
        }
    }

    public Action clawWristDown() {
        return new ClawWristDown();
    }

    public class ClawWristSet implements Action {
        double position = 0;
        public ClawWristSet(double position) {
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawWrist.setPosition(position);
            return false;
        }
    }

    public Action clawWristSet(double position) {
        return new ClawWristSet(position);
    }
}

