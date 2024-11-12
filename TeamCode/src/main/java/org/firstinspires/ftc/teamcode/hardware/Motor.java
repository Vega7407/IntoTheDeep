package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Motor {
    DcMotorEx internal;
    public static final double CPR_312 = 537.7;
    public static final double CPI_312_96 = CPR_312 / (2 * Math.PI * 96);

    public static final double CPR_84 = 1993.6;


    public Motor(DcMotorEx dcMotorEx) {
        this.internal = dcMotorEx;
        reset();
    }

    public Motor(HardwareMap hwMap, String name) {
        this(hwMap.get(DcMotorEx.class, name));
    }

    public DcMotorEx getInternal() {
        return internal;
    }

    public double getCurrent() {
        return internal.getCurrent(CurrentUnit.AMPS);
    }

    public double getPosition() {
        return internal.getCurrentPosition();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        internal.setDirection(direction);
    }

    public void setPower(double power) {
        internal.setPower(power);
    }

    public void reverse() {
        switch (internal.getDirection()) {
            case FORWARD:
                setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case REVERSE:
                setDirection(DcMotorSimple.Direction.FORWARD);
                break;
        }
    }

    public static Motor reversed(Motor motor) {
        motor.reverse();
        return motor;
    }

    public void reset() {
        internal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        internal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runToPosition(int target, double power) {
        internal.setTargetPosition(target);
        internal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        internal.setPower(power);

        while (internal.isBusy()) {

        }

        //internal.setPower(0);
        //reset();
    }
}
