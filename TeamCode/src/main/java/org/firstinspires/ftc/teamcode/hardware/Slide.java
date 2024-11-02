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
        slide.runToPosition((int) Math.round(2000), 1.0);
    }

    public void retractSlide () {
        slide.runToPosition((int) Math.round(-1900), 1.0);
    }

}
