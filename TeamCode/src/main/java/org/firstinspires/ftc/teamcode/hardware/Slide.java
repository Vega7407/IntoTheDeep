package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Slide {
    public Motor slide;

    public Slide (HardwareMap hwMap) {
        slide = new Motor(hwMap.get(DcMotorEx.class, "slides"));
    }

    public void extendSlide () {
        slide.runToPosition((int) 2650, 0.69);
    }

    public void retractSlide () {
        slide.runToPosition((int) 0, 0.85);
        slide.runToPosition((int) 0, 0.85); 
        slide.setPower(0);
        slide.reset();
    }

    public void setPower (double power) {
        slide.setPower(power);
    }

    public void checkPos() {
    }

    public int getPosition() {
        return slide.getCurrentPosition();
    }
}
