package org.firstinspires.ftc.teamcode.hardware;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MecanumDriveLocalizer {
    public static final double trackWidth = 16;
    public static final double inPerTick = Motor.getCPI_312_96();
    public static final MecanumKinematics kinematics = new MecanumKinematics(
            trackWidth, 1);

    public final Encoder leftFront, leftBack, rightBack, rightFront;
    public final IMU imu;

    private double lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
    private Rotation2d lastHeading;
    private boolean initialized;

    public MecanumDriveLocalizer(Chassis chassis) {
        leftFront = new OverflowEncoder(new RawEncoder(chassis.frontLeft));
        leftBack = new OverflowEncoder(new RawEncoder(chassis.backLeft));
        rightBack = new OverflowEncoder(new RawEncoder(chassis.backRight));
        rightFront = new OverflowEncoder(new RawEncoder(chassis.frontRight));

        imu = chassis.lazyImu.get();

        // TODO: reverse encoders if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
        PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
        PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
        PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        if (!initialized) {
            initialized = true;

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        double headingDelta = heading.minus(lastHeading);
        Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                new DualNum<Time>(new double[]{
                        (leftFrontPosVel.position - lastLeftFrontPos),
                        leftFrontPosVel.velocity,
                }).times(inPerTick),
                new DualNum<Time>(new double[]{
                        (leftBackPosVel.position - lastLeftBackPos),
                        leftBackPosVel.velocity,
                }).times(inPerTick),
                new DualNum<Time>(new double[]{
                        (rightBackPosVel.position - lastRightBackPos),
                        rightBackPosVel.velocity,
                }).times(inPerTick),
                new DualNum<Time>(new double[]{
                        (rightFrontPosVel.position - lastRightFrontPos),
                        rightFrontPosVel.velocity,
                }).times(inPerTick)
        ));

        lastLeftFrontPos = leftFrontPosVel.position;
        lastLeftBackPos = leftBackPosVel.position;
        lastRightBackPos = rightBackPosVel.position;
        lastRightFrontPos = rightFrontPosVel.position;

        lastHeading = heading;

        return new Twist2dDual<>(
                twist.line,
                DualNum.cons(headingDelta, twist.angle.drop(1))
        );
    }

    public static boolean withinTolerance(Pose2d a, Pose2d b, double tolerance) {
        return abs(a.position.x - b.position.x) < tolerance &&
                abs(a.position.y - b.position.y) < tolerance &&
                abs(a.heading.minus(b.heading)) < tolerance;

    }
}
