package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.PI
import kotlin.math.round


class Motor(private val internal: DcMotorEx) : DcMotorEx by internal {


    init {
        val controller = internal.controller
        internal.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        reset()
    }

    constructor(name: String, hwMap: HardwareMap) : this(hwMap[DcMotorEx::class.java, name])

    operator fun invoke(): DcMotorEx {
        return internal
    }

    val current: Double = internal.getCurrent(CurrentUnit.AMPS)
    val position get() = internal.currentPosition

    fun runToPosition(target: Int, power: Double) {
        this.targetPosition = target
        this.mode = DcMotor.RunMode.RUN_TO_POSITION
        this.power = power

        while (this.isBusy) {
            //weewoo
        }
    }

    fun reverse() = when (direction) {
        DcMotorSimple.Direction.FORWARD -> internal.direction = DcMotorSimple.Direction.REVERSE
        DcMotorSimple.Direction.REVERSE -> internal.direction = DcMotorSimple.Direction.FORWARD
        null -> internal.direction = DcMotorSimple.Direction.FORWARD
    }

    fun reset() {
        internal.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        internal.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }


    inner class PIDFAction ()

    companion object {
        @JvmStatic val CPR_312 = 537.7
        @JvmStatic val CPR_435 = 384.5
        @JvmStatic val CPR_84 = 1993.6

        @JvmStatic val CPI_312_96 = round((1 / (96 * PI)) * CPR_312 * 25.4).toInt()
        @JvmStatic val CPI_312_104 = round((1 / (104 * PI)) * CPR_312 * 25.4).toInt()
        @JvmStatic val CPI_435_104 = round((1 / (104 * PI)) * CPR_435 * 25.4).toInt()

        @JvmStatic val TPD_84 = round(CPR_84/360)

        fun reversed(motor: Motor): Motor {
            motor.reverse()
            return motor
        }

        fun reverse(motor: DcMotorEx) = when (motor.direction) {
            DcMotorSimple.Direction.FORWARD -> motor.direction = DcMotorSimple.Direction.REVERSE
            DcMotorSimple.Direction.REVERSE -> motor.direction = DcMotorSimple.Direction.FORWARD
            null -> motor.direction = DcMotorSimple.Direction.FORWARD
        }

        fun reversed(motor: DcMotorEx) : DcMotorEx {
            reverse(motor)
            return motor
        }
    }

}

