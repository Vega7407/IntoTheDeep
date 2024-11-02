package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Chassis;

@Autonomous
public class AutoMovement extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Chassis bobot = new Chassis(hardwareMap);
        waitForStart();
        bobot.frontLeft.runToPosition(12345678, 1);
        bobot.frontRight.runToPosition(12345678, 1);
        bobot.backLeft.runToPosition(12345678, 1);
        bobot.backRight.runToPosition(12345678, 1);
        sleep(3500);
        bobot.frontLeft.runToPosition(12345678, -1);
        bobot.frontRight.runToPosition(12345678, -1);
        bobot.backLeft.runToPosition(12345678, -1);
        bobot.backRight.runToPosition(12345678, -1);
    }
}
