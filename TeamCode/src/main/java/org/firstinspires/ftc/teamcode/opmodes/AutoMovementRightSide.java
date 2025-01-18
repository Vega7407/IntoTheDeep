package org.firstinspires.ftc.teamcode.opmodes;

import static android.os.SystemClock.sleep;
import static java.lang.Math.round;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.hardware.Chassis;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.dairy.pasteurized.PasteurizedGamepad;
import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import page.j5155.expressway.ftc.motion.FeedforwardFun;
import page.j5155.expressway.ftc.motion.PIDFController;

@Config
@TeleOp
public class AutoMovementRightSide extends LinearOpMode  {
    SDKGamepad gp1 = new SDKGamepad(gamepad1);

    TwoPointServo claw = new TwoPointServo(0.18, 0, 1, "claw", hardwareMap);;
    TwoPointServo clawWrist = new TwoPointServo(0.1, 0.055, 0, "clawWrist", hardwareMap);
    Slide slides = new Slide(hardwareMap);;
    Motor slideMotor = new Motor(hardwareMap.get(DcMotorEx.class, "slideMotor"));;
    Chassis bobot = new Chassis(hardwareMap);;
    PIDFController controller;
    boolean clawToggle;
    boolean clawWristToggle;
    public static double p = -0.002, i = 0, d = 0.0001;
    public static double normalF = 0.01;
    public static double zeroF = 0.12;
    public static double f = normalF;
    public static int target = 0;
    private final double ticks_in_degree = round(1993.6 / 360);
    int targetTicks = (int) (10 * round(1993.6 / 360)); // Convert rotations to encoder ticks
    public static PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients(p,i,d);

    @Override
    public void runOpMode() throws InterruptedException {
        slideMotor.reverse();
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw.positionA();
        clawWrist.positionB();

        wait(500);
        bobot.setMotorPowers(1, 1, 1, 1);

        // Set target position for each motor
        bobot.frontLeft.setTargetPosition(targetTicks);
        bobot.frontRight.setTargetPosition(targetTicks);
        bobot.backLeft.setTargetPosition(targetTicks);
        bobot.backRight.setTargetPosition(targetTicks);

        // Set motors to RUN_TO_POSITION mode
        bobot.frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bobot.frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bobot.backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bobot.backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Start motors with the specified power
        bobot.frontLeft.setPower(1);
        bobot.frontRight.setPower(1);
        bobot.backLeft.setPower(1);
        bobot.backRight.setPower(1);

        // Wait until all motors reach their target
        while (opModeIsActive() &&
                (bobot.frontLeft.isBusy() || bobot.frontRight.isBusy() ||
                        bobot.backLeft.isBusy() || bobot.backRight.isBusy())) {
            telemetry.addData("Front Left Position", bobot.frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Position", bobot.frontRight.getCurrentPosition());
            telemetry.addData("Back Left Position", bobot.backLeft.getCurrentPosition());
            telemetry.addData("Back Right Position", bobot.backRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motors
        bobot.frontLeft.setPower(0);
        bobot.frontRight.setPower(0);
        bobot.backLeft.setPower(0);
        bobot.backRight.setPower(0);

        // Reset motor modes to use encoders
        bobot.frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bobot.frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bobot.backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bobot.backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}

