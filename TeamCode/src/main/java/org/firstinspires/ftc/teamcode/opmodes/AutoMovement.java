package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Motor.COUNTS_PER_INCH;

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
        bobot.setPosition((int)(COUNTS_PER_INCH * 20), 1); //11.87in circumference, 573.3 = 1 rotation
    }
}
