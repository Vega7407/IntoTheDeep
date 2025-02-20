package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Chassis;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class GoToAPlaceLeft extends LinearOpMode {


    @Override
    public void runOpMode() {

        Chassis bobot = new Chassis(hardwareMap);
//        MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        ClawWrist clawWrist = new ClawWrist(hardwareMap);
        Action move = bobot.drive.actionBuilder(new Pose2d(13.0, -61.0, -Math.PI/2))
                .strafeTo(new Vector2d(10.2, -38.2))
                .waitSeconds(1)
                .setTangent(Math.PI/2)
                .splineTo(new Vector2d(46.5, -9.9), -Math.PI/2)
                .strafeTo(new Vector2d(46.5, -60.0))
                .strafeTo(new Vector2d(46.5, -9.9))
                .strafeTo(new Vector2d(55.5, -9.9))
                .strafeTo(new Vector2d(55.5, -60.0))
                .strafeTo(new Vector2d(55.5, -9.9))
                .strafeTo(new Vector2d(62.3 , -9.9))
                .strafeTo(new Vector2d(62.3, -62.0))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(2.3, -38.1))
                .strafeTo(new Vector2d(3.2, -33.5))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(46.5, -62.0))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(2.3, -38.1))
                .strafeTo(new Vector2d(3.2, -33.5))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(46.5, -62.0))
                .build();
//        Action traj = robot.actionBuilder(robot.localizer.getPose())
//                .strafeTo(new Vector2d(40, -10))
//                .stopAndAdd(clawWrist.clawWristUp())
//                .strafeTo(new Vector2d(30, -10))
//                .waitSeconds(.5)
//                .stopAndAdd(claw.openClaw())
//                .strafeTo(new Vector2d(25, 40))
//                .strafeTo(new Vector2d(50, 50))
//                .turn(-Math.PI/2)
//                .strafeTo(new Vector2d(45, 15))
//                .stopAndAdd(arm.setTarget(0))
//                .build();
//
//        Action up = robot.actionBuilder(robot.localizer.getPose())
//                .stopAndAdd(arm.setTarget(300))
//                .waitSeconds(1)
//                .build();
//        Action down = robot.actionBuilder(robot.localizer.getPose())
//                .stopAndAdd(arm.runArm())
//                .waitSeconds(1)
//                .build();
//        Action mid = robot.actionBuilder(robot.localizer.getPose())
//                .waitSeconds(1)
//                .stopAndAdd(arm.runArm())
//                .waitSeconds(1)
//                .build();
//
//        Action wait = robot.actionBuilder(robot.localizer.getPose())
//                .waitSeconds(1)
//                .build();
//
////        Action back = robot.actionBuilder(robot.localizer.getPose())
////                .strafeTo(new Vector2d(10, -30))
////                .strafeTo(new Vector2d(5, -50))
////                .strafeTo(new Vector2d(20, 10))
////                .build();
//        Action push = robot.actionBuilder(robot.localizer.getPose())
//                .strafeTo(new Vector2d(20, 55))
//                .strafeTo(new Vector2d(55, 65))
//                .strafeTo(new Vector2d(55, 85))
//                .strafeTo(new Vector2d(5, 85))  // first one away
//                .strafeTo(new Vector2d(62, 85))
//                .strafeTo(new Vector2d(62, 110))
//                .strafeTo(new Vector2d(15, 110)) // second one away
//                .strafeTo(new Vector2d(68, 110))
//                .strafeTo(new Vector2d(68, 133))
//                .strafeTo(new Vector2d(2, 133))
//                .build();

//        Actions.runBlocking(claw.closeClaw());
//
        waitForStart();
        Actions.runBlocking(move);
//
//        Actions.runBlocking(new ParallelAction(arm.runArm(), new SequentialAction(arm.setTarget(500), wait, arm.setTarget(200))));

//        Actions.runBlocking(new ParallelAction(arm.setTarget(0), arm.runArm(), arm.setTarget(0), new SequentialAction(arm.setTarget(0), back)));



    }
}
