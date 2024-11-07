package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Chassis {
    public Motor frontLeft;
    public Motor frontRight;
    public Motor backLeft;
    public Motor backRight;


    public Chassis(HardwareMap hwMap){
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
    public void setPosition(int distance, double power){
        frontLeft.runToPosition(distance, power);
        frontRight.runToPosition(distance, power);
        backLeft.runToPosition(distance, power);
        backRight.runToPosition(distance, power);
    }
}
