package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class TwoPointServo {
    public ServoImplEx claw;
    private final double pointA;
    private final double pointB;
    private final double pointC;
    public TwoPointServo (double pA, double pB, double pC, String aName, HardwareMap hwMap) {
        pointA = pA;
        pointB = pB;
        pointC = pC;
        claw = hwMap.get(ServoImplEx.class, aName);
        claw.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void positionA () {
        claw.setPosition(pointA);
    }

    public void positionB () {
        claw.setPosition(pointB);
    }

    public void positionC () {
        claw.setPosition(pointC);
    }

    public double getPosition() {
        return claw.getPosition();
    }

    public void setPosition(double position) {
        claw.setPosition(position);
    }
}
