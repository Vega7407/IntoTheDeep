package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VegaTestHardwareRookie {
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public VegaTestHardwareRookie(HardwareMap hwMap){
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        backRight = hwMap.get(DcMotorEx.class, "backRight");
    }

    public void setMotorPowers(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    public void setMotorPowers(double FlPower, double FrPower, double BlPower, double BrPower){
        frontLeft.setPower(FlPower);
        frontRight.setPower(FrPower);
        backLeft.setPower(BlPower);
        backRight.setPower(BrPower);
    }

    public void setMotorPowers(double y, double x, double rx){
        frontLeft.setPower(y + x + rx);
        frontRight.setPower(y - x + rx);
        backLeft.setPower(y - x - rx);
        backRight.setPower(y + x - rx);
    }
}
