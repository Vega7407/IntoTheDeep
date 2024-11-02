package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

class Motor(val dcMotorEx: DcMotorEx) {
    //this can no longer be changed to java
    //testing

    init {
        dcMotorEx.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        reset()
    }

    constructor(name: String, hwMap: HardwareMap) : this(hwMap.get(DcMotorEx::class.java, name))

    operator fun invoke() : DcMotorEx {
        return dcMotorEx
    }

    val current get() = this().getCurrent(CurrentUnit.AMPS)
    val position get() = this().currentPosition

    fun runToPosition(target: Int, power: Double) {
        dcMotorEx.targetPosition = target
        dcMotorEx.mode = DcMotor.RunMode.RUN_TO_POSITION
        dcMotorEx.power = power

        while (dcMotorEx.isBusy) {

        }

        dcMotorEx.power = 0.0
        reset()
    }

    fun reverse() = when (dcMotorEx.direction) {
        DcMotorSimple.Direction.FORWARD -> dcMotorEx.direction = DcMotorSimple.Direction.REVERSE
        DcMotorSimple.Direction.REVERSE -> dcMotorEx.direction = DcMotorSimple.Direction.FORWARD
        null -> dcMotorEx.direction = DcMotorSimple.Direction.FORWARD
    }

    fun reset() {
        dcMotorEx.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        dcMotorEx.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    companion object {
        final val TICKS_PER_REV = 537.7

        fun reversed(motor: Motor) : Motor {
            motor.reverse()
            return motor
        }
    }
}