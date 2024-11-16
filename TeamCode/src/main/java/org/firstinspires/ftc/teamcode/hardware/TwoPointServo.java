package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class TwoPointServo {
    public ServoImplEx claw;
    private final double pointA;
    private final double pointB;

    public TwoPointServo (double pA, double pB, String aName, HardwareMap hwMap) {
        pointA = pA;
        pointB = pB;
        claw = hwMap.get(ServoImplEx.class, aName);
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void positionA () {
        claw.setPosition(pointA);
    }

    public void positionB () {
        claw.setPosition(pointB);
    }
}
