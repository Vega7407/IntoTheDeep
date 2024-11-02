package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Motor;

@Autonomous
public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        Motor frontfront = new Motor(frontLeft);
        waitForStart();
        telemetry.addData("current position", frontLeft.getCurrentPosition());
        telemetry.update();
        frontLeft.setPower(1.0);
        sleep(2500);
        frontLeft.setPower(0.0);
        telemetry.addData("current position", frontLeft.getCurrentPosition());
        telemetry.update();
        frontfront.runToPosition(2*538, 1.0);

    }
}
