package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Chassis;

import page.j5155.expressway.ftc.actions.ActionRunner;

@Autonomous
public class AutonTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double up = Math.PI/2;
        double left = Math.PI;
        double down = - Math.PI / 2;

        Pose2d blueRight = new Pose2d(13.0, 61.0, down);
        Pose2d blueLeft = new Pose2d(-13.0, 61.0, down);
        Pose2d redRight = new Pose2d(13.0, -61.0, up);
        Pose2d redLeft = new Pose2d(-13.0, -61.0, up);
        Pose2d zero = new Pose2d(0, 0, 0);

        Chassis bobot = new Chassis(hardwareMap);
        Action move = bobot.drive.actionBuilder(redRight)
                .strafeTo(new Vector2d(3.2, -34.2))
                .strafeTo(new Vector2d(26.3, -34.2))
                .setTangent(up)
                .splineTo(new Vector2d(46.5, -9.9), down)
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

        Action test = bobot.drive.actionBuilder(zero)
                .strafeTo(new Vector2d(24, 0))
                .build();

        waitForStart();

        Actions.runBlocking(test);
    }
}
