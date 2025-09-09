package org.firstinspires.ftc.teamcode.opmodes;

import static java.lang.Math.round;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Chassis;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.Slide;
import org.firstinspires.ftc.teamcode.hardware.TwoPointServo;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.pasteurized.SDKGamepad;
import page.j5155.expressway.ftc.motion.FeedforwardFun;
import page.j5155.expressway.ftc.motion.PIDFController;

@Config
@TeleOp
public class DecodeOpMode extends OpMode {

    // Drive
    Chassis bobot;

    // Mech Motors
    Motor intake;
    Motor shooter;
    Motor climb;

    // toggle booleans
    boolean flywheel = false;

    // Misc
    SDKGamepad gp1;

    PIDFController controller;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        gp1 = new SDKGamepad(gamepad1);
        bobot = new Chassis(hardwareMap);
        intake = new Motor("Intake", hardwareMap);
        shooter = new Motor("Shooter", hardwareMap);
        climb = new Motor("Climb", hardwareMap);
    }
    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // drive

        double y = gp1.leftStickY().state();
        double x = gp1.leftStickX().state();
        double rx = gp1.rightStickX().state();

        bobot.setMotorPowers(y, x, rx);

        // intake
        if (gp1.x().onTrue()) {
            intake.setPower(-0.8);
        }

        // outtake
        if (gp1.a().onTrue()) {
            flywheel = !flywheel;
        }

        if (flywheel) {
            double velocity = -shooter.getVelocity();

            if (velocity < 1500.0) {
                shooter.setPower(1);
                ;
            } else {
                shooter.setPower(0);
            }
        } else {
            shooter.setPower(0);
        }

        if (gp1.dpadUp().onTrue()) {
            climb.setPower(1);
        } else if (gp1.dpadDown().onTrue()) {
            climb.setPower(-1);
        } else {
            climb.setPower(0);
        }

        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);

        telemetry.addLine("VEGA 7407");
        telemetry.addLine("");
        telemetry.addData("frontLeft", bobot.frontLeft.getPower());
        telemetry.addData("backLeft", bobot.backLeft.getPower());
        telemetry.addData("backRight", bobot.backRight.getPower());
        telemetry.addData("frontRight", bobot.frontRight.getPower());
        telemetry.addLine("");
        telemetry.addData("frontLeft", bobot.frontLeft.getDirection());
        telemetry.addData("backLeft", bobot.backLeft.getCurrentPosition());
        telemetry.addData("backRight", bobot.backRight.getCurrentPosition());
        telemetry.addData("frontRight", bobot.frontRight.getPosition());
        telemetry.addData("rightStick", rx);
        telemetry.addLine("");
        telemetry.addData("Flywheel", flywheel ? "ON" : "OFF");
        telemetry.addData("Shooter Velocity", "%.2f", shooter.getVelocity());
        telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
        telemetry.update();
    }
}
