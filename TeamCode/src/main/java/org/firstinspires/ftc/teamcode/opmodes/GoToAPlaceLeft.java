package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class GoToAPlaceLeft extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        ClawWrist clawWrist = new ClawWrist(hardwareMap);
        Action traj = robot.actionBuilder(robot.localizer.getPose())
//                .strafeTo(new Vector2d(40, -10))
//                .stopAndAdd(clawWrist.clawWristUp())
//                .strafeTo(new Vector2d(30, -10))
//                .waitSeconds(.5)
//                .stopAndAdd(claw.openClaw())
                .strafeTo(new Vector2d(25, 40))
                .strafeTo(new Vector2d(50, 50))
                .turn(-Math.PI/2)
                .strafeTo(new Vector2d(45, 15))
                .build();

//        Action back = robot.actionBuilder(robot.localizer.getPose())
//                .strafeTo(new Vector2d(10, -30))
//                .strafeTo(new Vector2d(5, -50))
//                .strafeTo(new Vector2d(20, 10))
//                .build();
        Action push = robot.actionBuilder(robot.localizer.getPose())
                .strafeTo(new Vector2d(20, 55))
                .strafeTo(new Vector2d(55, 65))
                .strafeTo(new Vector2d(55, 85))
                .strafeTo(new Vector2d(5, 85))  // first one away
                .strafeTo(new Vector2d(62, 85))
                .strafeTo(new Vector2d(62, 110))
                .strafeTo(new Vector2d(15, 110)) // second one away
                .strafeTo(new Vector2d(68, 110))
                .strafeTo(new Vector2d(68, 133))
                .strafeTo(new Vector2d(2, 133))
                .build();

        Actions.runBlocking(claw.closeClaw());

        waitForStart();

        Actions.runBlocking(new ParallelAction(arm.runArm(), arm.setTarget(520)
                , new SequentialAction(arm.setTarget(520), traj)));

//        Actions.runBlocking(new ParallelAction(arm.setTarget(0), arm.runArm(), arm.setTarget(0), new SequentialAction(arm.setTarget(0), back)));



    }
}
