package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Chassis.blueRight;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Chassis;

@Autonomous
public class AutoMovementRightSide extends LinearOpMode {
    Chassis bobot;
    @Override
    public void runOpMode() throws InterruptedException {
        bobot = new Chassis(hardwareMap, blueRight);
        waitForStart();

    }
}
