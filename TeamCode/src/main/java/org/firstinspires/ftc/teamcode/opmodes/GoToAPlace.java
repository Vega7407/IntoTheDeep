package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.round;

import android.util.SparseArray;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
import kotlin.reflect.KParameter;
import page.j5155.expressway.core.actions.RaceParallelAction;
import page.j5155.expressway.ftc.motion.PIDFController;

@Autonomous
public class GoToAPlace extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        ClawWrist clawWrist = new ClawWrist(hardwareMap);
        Action traj = robot.actionBuilder(robot.localizer.getPose())
                .strafeTo(new Vector2d(45, 10))
                .stopAndAdd(new ParallelAction(clawWrist.clawWristDown()))
                .waitSeconds(1)
                .stopAndAdd(claw.openClaw())
                .strafeTo(new Vector2d(25, 10))
                .strafeTo(new Vector2d(15, 10))
                .build();

        Action back = robot.actionBuilder(robot.localizer.getPose())
                .strafeTo(new Vector2d(10, -30))
                .strafeTo(new Vector2d(5, -50))
                .strafeTo(new Vector2d(20, 10))
                .build();
        Action push = robot.actionBuilder(robot.localizer.getPose())
                .strafeTo(new Vector2d(20, -55))
                .strafeTo(new Vector2d(55, -65))
                .strafeTo(new Vector2d(55, -85))
                .strafeTo(new Vector2d(5, -85))  // first one away
                .strafeTo(new Vector2d(62, -85))
                .strafeTo(new Vector2d(62, -110))
                .strafeTo(new Vector2d(15, -110)) // second one away
                .strafeTo(new Vector2d(60, -110))
                .strafeTo(new Vector2d(60, -133))
                .strafeTo(new Vector2d(2, -133))
                .build();

        Actions.runBlocking(claw.closeClaw());

        waitForStart();

        Actions.runBlocking(new ParallelAction(arm.runArm(), arm.setTarget(520)
                , new SequentialAction(arm.setTarget(520), traj, push)));

//        Actions.runBlocking(new ParallelAction(arm.setTarget(0), arm.runArm(), arm.setTarget(0), new SequentialAction(arm.setTarget(0), back)));



    }
}
