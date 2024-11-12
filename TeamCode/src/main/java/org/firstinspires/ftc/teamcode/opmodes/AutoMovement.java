package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Motor.CPI_312_96;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Chassis;

@Autonomous
public class AutoMovement extends LinearOpMode {
    Chassis bobot;
    @Override
    public void runOpMode() throws InterruptedException {
        bobot = new Chassis(hardwareMap);
        waitForStart();
        bobot.setPosition((int)(CPI_312_96 * 20), 1); //11.87in circumference, 573.3 = 1 rotation
    }
}
