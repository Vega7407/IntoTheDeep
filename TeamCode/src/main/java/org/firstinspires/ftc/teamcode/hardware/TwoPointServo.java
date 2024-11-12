package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TwoPointServo {
    public Servo claw;
    public Servo clawWrist;
    private final double pointA;
    private final double pointB;

    public TwoPointServo (double pA, double pB, String aName, HardwareMap hwMap) {
        pointA = pA;
        pointB = pB;
        claw = hwMap.get(Servo.class, aName);
    }

    public void positionA () {
        claw.setPosition(pointA);
    }

    public void positionB () {
        claw.setPosition(pointB);
    }
}
