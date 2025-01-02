package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import page.j5155.expressway.ftc.motion.PIDFController;

public class Chassis {
    public Motor frontLeft;
    public Motor frontRight;
    public Motor backLeft;
    public Motor backRight;
    private Pose2d position;


    public Chassis(HardwareMap hwMap){
        this(hwMap, new Pose2d(0.0, 0.0, 0.0));
    }

    public Chassis(HardwareMap hwMap, Pose2d beginPose) {
        DcMotorEx frontL = hwMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontR = hwMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backL = hwMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backR = hwMap.get(DcMotorEx.class, "backRight");
        frontLeft = new Motor(frontL);
        frontRight = new Motor(frontR);
        backLeft = new Motor(backL);
        backRight = new Motor(backR);

        this.position = beginPose;
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

    public Pose2d getPosition() {
        return position;
    }

    //Allows the bobot to move to a certain position on the field using  PID
    public void moveToPoint(Pose2d point) {
        double p = -0.002, i = 0, d = 0.0001;
        PIDFController xCont, yCont, hCont;
        PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p,i,d);
        xCont = new PIDFController(coefficients);
        yCont = new PIDFController(coefficients);
        hCont = new PIDFController(coefficients);
        xCont.setTargetPosition((int) point.position.x);
        yCont.setTargetPosition((int) point.position.y);
        hCont.setTargetPosition((int) point.heading.toDouble());
        while (!this.position.equals(point)) {
            double xPow = xCont.update(this.position.position.x);
            double yPow = xCont.update(this.position.position.y);
            double hPow = xCont.update(this.position.heading.toDouble());
            setMotorPowers(yPow, xPow, hPow);
        }
    }
}
