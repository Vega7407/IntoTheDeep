package org.firstinspires.ftc.teamcode.opmodes;

import static android.os.SystemClock.sleep;
import static java.lang.Math.round;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.ejml.dense.row.mult.VectorVectorMult_CDRM;
import org.firstinspires.ftc.teamcode.hardware.Chassis;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import dev.frozenmilk.dairy.pasteurized.PasteurizedGamepad;
import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import page.j5155.expressway.ftc.motion.FeedforwardFun;
import page.j5155.expressway.ftc.motion.PIDFController;

@Config
@Autonomous
public class AutoMovementRightSide extends LinearOpMode  {
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
        double up = Math.PI/2;
        double left = Math.PI;
        double down = - Math.PI / 2;

        Pose2d blueRight = new Pose2d(13.0, 61.0, down);
        Pose2d blueLeft = new Pose2d(-13.0, 61.0, down);
        Pose2d redRight = new Pose2d(13.0, -61.0, down);
        Pose2d redLeft = new Pose2d(-13.0, -61.0, up);
        Pose2d zero = new Pose2d(0, 0, 0);

        Claw claw = new Claw(hardwareMap);
        ClawWrist clawWrist = new ClawWrist(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Chassis bobot = new Chassis(hardwareMap);
        SlideAuto slides = new SlideAuto(hardwareMap);

        Action parkRight = bobot.drive.actionBuilder(redRight)
                .strafeTo(new Vector2d(3.2, -34.2))
                .build();

        Action clip = bobot.drive.actionBuilder(new Pose2d(0, 0, 0))
                .stopAndAdd(new SequentialAction(new ParallelAction(clawWrist.clawWristSet(0.72), arm.setTarget(360))))
                .waitSeconds(0.5)
                .stopAndAdd( slides.prepSlide())
                .waitSeconds(1)
                .stopAndAdd(slides.clipSlide())
                .waitSeconds(0.3)
                .stopAndAdd(new ParallelAction(claw.openClaw(), slides.retractSlide()))
                .build();

        Action wall = bobot.drive.actionBuilder(new Pose2d(0, 0, 0))
                .stopAndAdd(new ParallelAction(arm.setTarget(210), clawWrist.clawWristSet(0.34), claw.openClaw()))
                .waitSeconds(2)
                .stopAndAdd(claw.closeClaw())
                .build();

        Action test = bobot.drive.actionBuilder(new Pose2d(0, 0, 0))
                .stopAndAdd(slides.clipSlide())
                .waitSeconds(1)
                .stopAndAdd(slides.retractSlide())
                .build();

        Action motion = bobot.drive.actionBuilder(redRight)
                .strafeTo(new Vector2d(3.2, -32.2))
//                .strafeTo(new Vector2d(32.3, -31.2))
//                .setTangent(up)
//                .strafeTo(new Vector2d(36.3, -2))
//                .turn(Math.PI * 0.93)
//                .strafeTo(new Vector2d(42, -2))
//                .strafeTo(new Vector2d(40, -60.0))
//                .strafeTo(new Vector2d(38, -2))
//                .strafeTo(new Vector2d(44.5, -2))
//                .strafeTo(new Vector2d(44.5, -60.0))
//                .strafeTo(new Vector2d(44.5, -2))
//                .strafeTo(new Vector2d(50.3 , -2))
//                .strafeTo(new Vector2d(50.3, -62.0))
//                .waitSeconds(0.2)
//                .strafeTo(new Vector2d(2.3, -38.1))
//                .strafeTo(new Vector2d(3.2, -33.5))
//                .waitSeconds(0.2)
//                .strafeTo(new Vector2d(46.5, -62.0))
//                .waitSeconds(0.2)
//                .strafeTo(new Vector2d(2.3, -38.1))
//                .strafeTo(new Vector2d(3.2, -33.5))
//                .waitSeconds(0.2)
//                .strafeTo(new Vector2d(46.5, -62.0))
                .build();

        Actions.runBlocking(claw.closeClaw());

        waitForStart();

//        Actions.runBlocking(new ParallelAction(arm.runArm(),new SequentialAction(motion, clip)));
//        Actions.runBlocking(new SequentialAction(new ParallelAction(clawWrist.clawWristUp(), slides.clipSlide()), new ParallelAction(claw.openClaw(), slides.retractSlide())));
        Actions.runBlocking(new ParallelAction(arm.runArm(), new ParallelAction(motion, new SequentialAction(clip, wall))));
//        Actions.runBlocking(clip);

    }
}

