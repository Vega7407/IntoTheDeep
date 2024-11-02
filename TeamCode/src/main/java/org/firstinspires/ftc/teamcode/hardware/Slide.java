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
        slide.runToPosition((int) Math.round(1.5*Motor.COUNTS_PER_REV), 1.0);
    }

    public void retractSlide () {
        slide.runToPosition((int) Math.round(-1.5*Motor.COUNTS_PER_REV), 1.0);
    }

}
