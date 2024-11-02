package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ChassisMotor {
    public Motor frontLeft;
    public Motor frontRight;
    public Motor backLeft;
    public Motor backRight;


    public ChassisMotor(HardwareMap hwMap){
        DcMotorEx frontL = hwMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontR = hwMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backL = hwMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backR = hwMap.get(DcMotorEx.class, "backRight");
        Motor frontLeft = new Motor(frontL);
        Motor frontRight = new Motor(frontR);
        Motor backLeft = new Motor(backL);
        Motor backRight = new Motor(backR);
    }

    public void setMotorPowers(double power){

        frontLeft.invoke().setPower(power);
        frontRight.invoke().setPower(power);
        backLeft.invoke().setPower(power);
        backRight.invoke().setPower(power);
    }

    public void setMotorPowers(double FlPower, double FrPower, double BlPower, double BrPower){
        frontLeft.invoke().setPower(FlPower);
        frontRight.invoke().setPower(FrPower);
        backLeft.invoke().setPower(BlPower);
        backRight.invoke().setPower(BrPower);
    }

    public void setMotorPowers(double y, double x, double rx){
        frontLeft.invoke().setPower(y + x + rx);
        frontRight.invoke().setPower(y - x + rx);
        backLeft.invoke().setPower(y - x - rx);
        backRight.invoke().setPower(y + x - rx);
    }
}
