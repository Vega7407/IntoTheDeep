package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.round;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Chassis;

import page.j5155.expressway.ftc.motion.PIDFController;

@Config
@Autonomous
public class AutoMovementLeftSide extends LinearOpMode  {
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
        Pose2d redLeft = new Pose2d(-15.0, -61.0, up);
        Pose2d zero = new Pose2d(0, 0, 0);

        Claw claw = new Claw(hardwareMap);
        ClawWrist clawWrist = new ClawWrist(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Chassis bobot = new Chassis(hardwareMap);
        SlideAuto slides = new SlideAuto(hardwareMap);

        Action parkRight = bobot.drive.actionBuilder(redRight)
                .strafeTo(new Vector2d(3.2, -34.2))
                .build();

        Action basket = bobot.drive.actionBuilder(redLeft)
                .stopAndAdd(new ParallelAction(arm.setTarget(375), clawWrist.clawWristDown()))
                .waitSeconds(0.3)
                .stopAndAdd(slides.extendSlide())
                .waitSeconds(0.15)
                .stopAndAdd(clawWrist.clawWristUp())
                .waitSeconds(0.23)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.4)
                .stopAndAdd(slides.retractSlide())
                .stopAndAdd(arm.setTarget(0))
                .build();

        Action basket1 = bobot.drive.actionBuilder(redLeft)
                .stopAndAdd(new ParallelAction(arm.setTarget(375), clawWrist.clawWristDown()))
                .waitSeconds(0.3)
                .stopAndAdd(slides.extendSlide())
                .waitSeconds(0.15)
                .stopAndAdd(clawWrist.clawWristUp())
                .waitSeconds(0.23)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(2)
                .stopAndAdd(slides.retractSlide())
                .stopAndAdd(arm.setTarget(0))
                .build();

        Action basket2 = bobot.drive.actionBuilder(redLeft)
                .stopAndAdd(new ParallelAction(arm.setTarget(375), clawWrist.clawWristDown()))
                .waitSeconds(0.3)
                .stopAndAdd(slides.extendSlide())
                .waitSeconds(0.15)
                .stopAndAdd(clawWrist.clawWristUp())
                .waitSeconds(0.23)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.7)
                .stopAndAdd(slides.retractSlide())
                .stopAndAdd(arm.setTarget(0))
                .build();

        Action basket3 = bobot.drive.actionBuilder(redLeft)
                .stopAndAdd(new ParallelAction(arm.setTarget(375), clawWrist.clawWristDown()))
                .waitSeconds(0.3)
                .stopAndAdd(slides.extendSlide())
                .waitSeconds(0.15)
                .stopAndAdd(clawWrist.clawWristUp())
                .waitSeconds(0.23)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(0.7)
                .stopAndAdd(slides.retractSlide())
                .afterTime(0.1, arm.setTarget(0))
                .build();

        Action floor = bobot.drive.actionBuilder(redLeft)
                .stopAndAdd(new ParallelAction( clawWrist.clawWristDown(), slides.sample1()))
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(0.4)
                .stopAndAdd(slides.retractSlide())
                .build();

        Action actionAuton = bobot.drive.actionBuilder(redLeft)
                .afterTime(1.8, new ParallelAction(arm.setTarget(375), clawWrist.clawWristDown()))
                .afterTime(2.2, slides.extendSlide())
                .afterTime(2.4, clawWrist.clawWristUp())
                .afterTime(3.3, claw.openClaw())
                .afterTime(4.8, slides.retractSlide())
                .afterTime(6, arm.setTarget(0))
                .afterTime(7, floor)
                .afterTime(10.3, new ParallelAction(arm.setTarget(375), clawWrist.clawWristDown()))
                .afterTime(10.6, slides.extendSlide())
                .afterTime(10.75, clawWrist.clawWristUp())
                .afterTime(11, claw.openClaw())
                .afterTime(12.2, slides.retractSlide())
                .afterTime(13.8, arm.setTarget(0))
                .afterTime(16.5, floor)
                .afterTime(20, basket2)
                .afterTime(25, floor)
                .afterTime(28, basket3)
                .build();

        Action moveAuton = bobot.drive.actionBuilder(redLeft)
                // go to basket
                .strafeToSplineHeading(new Vector2d(-57.5, -52.2), up * 0.55)
                .waitSeconds(2)
                // go to right sample
                .strafeToSplineHeading(new Vector2d(-50.6, -44), up)
                .waitSeconds(1.6)
                // go to basket
                .strafeToSplineHeading(new Vector2d(-56.5, -50), up * 0.5)
                .waitSeconds(6)
                // go to middle sample
                .strafeToSplineHeading(new Vector2d(-58.5, -41.5), up)
                .waitSeconds(1.5)
                // go to basket
                .strafeToSplineHeading(new Vector2d(-56, -48), up * 0.5)
                .waitSeconds(0.5)
                // go to left sample
                .strafeToSplineHeading(new Vector2d(-57.6, -37.0), up * 1.4)
                .waitSeconds(1.5)
                // go to basket
                .strafeToSplineHeading(new Vector2d(-58, -52.2), up * 0.5)
                .waitSeconds(0.5)
                // level 1 ascent
                .strafeToSplineHeading(new Vector2d(-24.1, -4.2), -Math.PI)
                .waitSeconds(1.5)
                .stopAndAdd(arm.setTarget(360))
                .build();

        Actions.runBlocking(new ParallelAction(claw.closeClaw(), clawWrist.clawWristSet(0.85)));

        waitForStart();

//        Actions.runBlocking(new ParallelAction(arm.runArm(),new SequentialAction(motion, clip)));
//        Actions.runBlocking(new SequentialAction(new ParallelAction(clawWrist.clawWristUp(), slides.clipSlide()), new ParallelAction(claw.openClaw(), slides.retractSlide())));
//        Actions.runBlocking(motion);
//        Actions.runBlocking(new ParallelAction(arm.runArm(), new ParallelAction(new SequentialAction(grabColor, grabAndDrop), clip)));
        Actions.runBlocking(new ParallelAction(arm.runArm(), moveAuton, actionAuton));
//            Actions.runBlocking(grabAndDrop);
//        Actions.runBlocking(clip);
//        Actions.runBlocking(new ParallelAction(arm.runArm(), arm.setTarget(260), clawWrist.clawWristSet(0.34)));

    };
}

