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
public class AutonTest extends OpMode {
    Chassis bobot;
    ActionRunner runner;
    Action move;


    public void init()  {
        Chassis bobot = new Chassis(hardwareMap);
        Action move = bobot.drive.actionBuilder(new Pose2d(0.0, 0.0, 0))
                .waitSeconds(1)
                .turn(Math.PI/2)
                .build();
        runner = new ActionRunner();
    }

    public void start() {
        runner.runAsync(move);
    }

    public void loop() {
        telemetry.addData("Robot pose", bobot.getPose());
        bobot.moveToPoint(new Pose2d(24, 0, 0));
        runner.updateAsync();
        telemetry.update();
    }
}
