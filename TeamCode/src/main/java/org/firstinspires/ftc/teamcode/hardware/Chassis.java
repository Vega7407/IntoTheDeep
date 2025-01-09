package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.MecanumDriveLocalizer.withinTolerance;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.Time;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import page.j5155.expressway.ftc.motion.PIDFController;

public class Chassis {
    public static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    public static Pose2d blueRight = new Pose2d(36.0, 60.0, -Math.PI/2);
    public static Pose2d blueLeft = new Pose2d(-26.0, 60.0, -Math.PI/2);
    public static Pose2d redRight = new Pose2d(26.0, -60.0, Math.PI/2);
    public static Pose2d redLeft = new Pose2d(-36.0, -60.0, Math.PI/2);
    public Motor frontLeft;
    public Motor frontRight;
    public Motor backLeft;
    public Motor backRight;
    private Pose2d pose;
    public LazyImu lazyImu;
    private MecanumDriveLocalizer localizer;
    private PoseVelocity2d velocity;
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

        this.pose = beginPose;
        this.velocity = new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);

        this.lazyImu = new LazyImu(hwMap, "imu", new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));

        this.localizer = new MecanumDriveLocalizer(this);
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

    public Pose2d getPose() {
        return pose;
    }

    public void updatePoseEstimate() {
        Twist2dDual<Time> twist = localizer.update();
        pose = pose.plus(twist.value());
        velocity = twist.velocity().value();
    }

    // pose is 1. the vector of bot position (x and y) and 2. the rotation
    // twist is difference between two poses

    /**
     * Allows the bobot to move to a certain position on the field using PID;
     * Note that this is blocking
     * (as in this method does not end until the robot completes its movement)
     */
    public void moveToPoint(Pose2d point) {
        double p = 0.002, i = 0, d = 0.0001;
        PIDFController xCont, yCont, hCont;
        PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p,i,d);

        xCont = new PIDFController(coefficients);
        yCont = new PIDFController(coefficients);
        hCont = new PIDFController(coefficients);

        xCont.setTargetPosition((int) point.position.x);
        yCont.setTargetPosition((int) point.position.y);
        hCont.setTargetPosition((int) point.heading.toDouble());

        while (withinTolerance(this.pose, point, 4)) {
            double xPow = xCont.update(this.pose.position.x);
            double yPow = xCont.update(this.pose.position.y);
            double hPow = xCont.update(this.pose.heading.toDouble());
            setMotorPowers(yPow, xPow, hPow);

            updatePoseEstimate();
        }
    }
}
