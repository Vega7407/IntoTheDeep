package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Slide {
    public Motor slide;

    public Slide (HardwareMap hwMap) {
        DcMotorEx slideMotor = hwMap.get(DcMotorEx.class, "slides");
        slide = new Motor(slideMotor);
    }

    public void extendSlide () {
        slide.runToPosition((int) 2000, 0.69);
    }

    public void retractSlide () {
        slide.runToPosition((int) 0, 0.69);
    }

    public void setSlide () {
        slide.runToPosition((int) 70, 1);
    }
}
