package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous
public class GoToAPlace extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive robot = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        Action traj = robot.actionBuilder(robot.localizer.getPose())
                .splineTo(new Vector2d(24.0, 24.0), Math.PI)
                .build();
        Action arm = robot.actionBuilder(robot.localizer.getPose())
                        .strafeTo(new Vector2d(10.0, 30.0))
                        .build();


        waitForStart();

        Actions.runBlocking(traj

        );
    }
}
