package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.control.PIDFController;
import org.firstinspires.ftc.teamcode.hardware.Motor;

@TeleOp
public class PIDTest extends OpMode {
    Motor arm;
    PIDFController controller;

    @Override
    public void init() {
        controller = new PIDFController(new PIDFController.PIDCoefficients(1, 0, 0.1));

        arm = new Motor(hardwareMap.get(DcMotorEx.class, "arm"));
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            controller.setTargetPosition((int) (45 * Motor.getTPD_84()));
        } else if (gamepad1.dpad_down) {
            controller.setTargetPosition(0);
        }

        double power = controller.update(arm.getPosition());
        double error = controller.getTargetPosition() - arm.getPosition();

        telemetry.addData("Target", controller.getTargetPosition());
        telemetry.addData("Position", arm.getPosition());
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);
        telemetry.update();

        arm.setPower(power);
    }
}
