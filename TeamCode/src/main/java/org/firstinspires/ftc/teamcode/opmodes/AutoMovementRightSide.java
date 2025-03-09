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
        Pose2d redRight = new Pose2d(9.0, -61.0, down);
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
                .stopAndAdd(new ParallelAction(clawWrist.clawWristUp(), arm.setTarget(340)))
                .waitSeconds(0.8)
                .stopAndAdd( slides.prepSlide())
                .waitSeconds(.15)
                .stopAndAdd(slides.clipSlide())
                .waitSeconds(0.05)
                .stopAndAdd(new SequentialAction(new ParallelAction(claw.openClaw(), slides.retractSlide()), clawWrist.clawWristUp()))
                .build();

        Action wall = bobot.drive.actionBuilder(new Pose2d(0, 0, 0))
                .stopAndAdd(new ParallelAction(arm.setTarget(210), clawWrist.clawWristSet(0.34), claw.openClaw()))
                .waitSeconds(2)
                .stopAndAdd(claw.closeClaw())
                .build();

        Action test = bobot.drive.actionBuilder(new Pose2d(0, 0, 0))
                .stopAndAdd(slides.clipSlide())
                .waitSeconds(0.1)
                .stopAndAdd(slides.retractSlide())
                .build();
        Action grabColor = bobot.drive.actionBuilder(redRight)
                .strafeToConstantHeading(new Vector2d(1.2, -38.8))
                .waitSeconds(1)
                .stopAndAdd(arm.setTarget(0))
                .setTangent(down)
                .splineTo(new Vector2d(42.5, -45), up * 1.1)
                .stopAndAdd(new ParallelAction(clawWrist.clawWristDown(), claw.openClaw()))
                .stopAndAdd(new SequentialAction(slides.sample1(), claw.closeClaw()))
                .waitSeconds(0.4)
                .turnTo(down)
//                .stopAndAdd( new ParallelAction(clawWrist.clawWristUp(), claw.openClaw(), slides.sample2()))
//                .turnTo(up)
//                .stopAndAdd(new ParallelAction(clawWrist.clawWristDown(), claw.openClaw()))
//                .waitSeconds(0.2)
//                .stopAndAdd(new SequentialAction(slides.sample1(), claw.closeClaw()))
//                .waitSeconds(0.4)
//                .turnTo(up * 1.5)
//                .turnTo(down)
                // drop #2

                // drop #1 done - 4 + 0

                .stopAndAdd( new ParallelAction(clawWrist.clawWristUp(), claw.openClaw(), slides.sample2()))
                .turnTo((up * 0.77))
                .stopAndAdd(new ParallelAction(clawWrist.clawWristDown(), claw.openClaw()))
                .waitSeconds(0.2)
                .stopAndAdd(slides.sample1())
                .waitSeconds(0.1)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.3)
                .stopAndAdd(slides.sample2())
                .turnTo(down)
                .stopAndAdd(new SequentialAction(arm.setTarget(250), clawWrist.clawWristSet(0.21), claw.openClaw()))
                // drop and dpad_right or up to wall

                .strafeTo(new Vector2d(49, -67))
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.3)
                .stopAndAdd(new ParallelAction(clawWrist.clawWristUp(), arm.setTarget(340)))
                .waitSeconds(0.2)
                .stopAndAdd( slides.prepSlide())
                .strafeToConstantHeading(new Vector2d(-4, -35.2))
                .stopAndAdd(slides.clipSlide())
                .stopAndAdd(new SequentialAction(new ParallelAction(claw.openClaw(), slides.retractSlide()), clawWrist.clawWristUp()))
                .stopAndAdd(new SequentialAction(claw.openClaw(), new ParallelAction(arm.setTarget(212), clawWrist.clawWristSet(0.21))))
                .strafeTo(new Vector2d(45, -78))
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.3)
                .stopAndAdd(new ParallelAction(clawWrist.clawWristUp(), arm.setTarget(340)))
                .waitSeconds(0.2)
                .stopAndAdd( slides.prepSlide())
                .strafeToConstantHeading(new Vector2d(-3, -35))
                .stopAndAdd(slides.clipSlide())
                .stopAndAdd(new SequentialAction(new ParallelAction(claw.openClaw(), slides.retractSlide()), clawWrist.clawWristUp()))
                .stopAndAdd(new SequentialAction(claw.openClaw(), new ParallelAction(arm.setTarget(212), clawWrist.clawWristSet(0.21))))
                .strafeTo(new Vector2d(45, -78))
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.3)
                .stopAndAdd(new ParallelAction(clawWrist.clawWristUp(), arm.setTarget(340)))
                .waitSeconds(0.2)
                .stopAndAdd( slides.prepSlide())
                .strafeToConstantHeading(new Vector2d(2, -35))
                .stopAndAdd(slides.clipSlide())
                .stopAndAdd(new SequentialAction(new ParallelAction(claw.openClaw(), slides.retractSlide()), clawWrist.clawWristUp()))
                .build();

        Action grabAndDrop = bobot.drive.actionBuilder(new Pose2d(0, 0, down))
//                // drop #1
//                .stopAndAdd( new ParallelAction(clawWrist.clawWristUp(), claw.openClaw(), slides.sample2()))
//                .turnTo(up)
//                .stopAndAdd(new ParallelAction(clawWrist.clawWristDown(), claw.openClaw()))
//                .waitSeconds(0.2)
//                .stopAndAdd(new SequentialAction(slides.sample1(), claw.closeClaw()))
//                .waitSeconds(0.4)
//                .turnTo(0)
//                // drop #2
//                .stopAndAdd( new ParallelAction(clawWrist.clawWristUp(), claw.openClaw(), slides.sample2()))
//                .turnTo((up * 0.86))
//                .stopAndAdd(new ParallelAction(clawWrist.clawWristDown(), claw.openClaw()))
//                .waitSeconds(0.2)
//                .stopAndAdd(new SequentialAction(slides.sample1(), claw.closeClaw()))
//                .waitSeconds(0.4)
//                .stopAndAdd(slides.retractSlide())
//                .turnTo(0)
                .stopAndAdd( claw.openClaw())
                .stopAndAdd(new ParallelAction(arm.setTarget(245), clawWrist.clawWristSet(0.21)))
                .strafeTo(new Vector2d(33, 0))
                .waitSeconds(1)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.5)
                .strafeTo(new Vector2d(-5, -65))
                .stopAndAdd(clip)
                .waitSeconds(3)
                .strafeTo(new Vector2d(30, 0))
                .waitSeconds(1.5)
                .stopAndAdd(wall)
                .strafeTo(new Vector2d(-5, -65))
                .stopAndAdd(wall)
//
//                .strafeTo(new Vector2d(0, 8))
//                .waitSeconds(0.2)
//
//                .stopAndAdd(new SequentialAction(slides.sample1(), claw.closeClaw()))
//                .waitSeconds(0.4)
//                .strafeTo(new Vector2d(32.5, 10))
//                .stopAndAdd( new ParallelAction(clawWrist.clawWristUp(), claw.openClaw()))
//                .strafeTo(new Vector2d(0, 0))
//                .stopAndAdd(clawWrist.clawWristDown())
//                .strafeTo(new Vector2d(-10, 13))
//                .waitSeconds(0.2)
//                .turn(-Math.PI/6)
//                .stopAndAdd(new SequentialAction( slides.sample1(), claw.closeClaw()))
//                .waitSeconds(0.4)
//                .splineTo(new Vector2d(25, 13), 0)
//                .stopAndAdd( new ParallelAction(clawWrist.clawWristUp(), claw.openClaw()))
//                .build();
//        Action motion = bobot.drive.actionBuilder(redRight)
//                .strafeTo(new Vector2d(5.2, -40.2))
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(5.3, -43))
//                .strafeTo(new Vector2d(43.3, -53.2))
//                .strafeTo(new Vector2d(43.3, -15))
//                .strafeTo(new Vector2d(56, -15))
//                .strafeTo(new Vector2d(56, -65.0))
//                .strafeTo(new Vector2d(56, -20))
//                .strafeTo(new Vector2d(70, -20))
//                .strafeTo(new Vector2d(70, -65.0))
                .build();

        Actions.runBlocking(new ParallelAction(claw.closeClaw(), clawWrist.clawWristSet(0.23)));

        waitForStart();

//        Actions.runBlocking(new ParallelAction(arm.runArm(),new SequentialAction(motion, clip)));
//        Actions.runBlocking(new SequentialAction(new ParallelAction(clawWrist.clawWristUp(), slides.clipSlide()), new ParallelAction(claw.openClaw(), slides.retractSlide())));
//        Actions.runBlocking(motion);
//        Actions.runBlocking(new ParallelAction(arm.runArm(), new ParallelAction(new SequentialAction(grabColor, grabAndDrop), clip)));
        Actions.runBlocking(new ParallelAction(arm.runArm(), new ParallelAction(new SequentialAction(grabColor), clip)));
//            Actions.runBlocking(grabAndDrop);
//        Actions.runBlocking(clip);
//        Actions.runBlocking(new ParallelAction(arm.runArm(), arm.setTarget(260), clawWrist.clawWristSet(0.34)));

    };
}

