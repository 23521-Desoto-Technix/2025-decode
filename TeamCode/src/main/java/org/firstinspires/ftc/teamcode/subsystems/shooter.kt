package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.abs

object Shooter : Subsystem {
    private val upperShooterMotor = MotorEx("upperShooter").brakeMode().reversed()
    private val lowerShooterMotor = MotorEx("lowerShooter").brakeMode()

    private var power = 0.0

    var speed = 0.0
    var targetSpeed = 0.0

    private const val SPEED_AT_MAX_POWER = 2_600
    private const val PROPORTIONAL_GAIN = 0.001
    private const val SPEED_TOLERANCE = 50

    override fun periodic() {
        speed = upperShooterMotor.state.velocity
        upperShooterMotor.power = power
        lowerShooterMotor.power = power
    }

    fun setSpeed(targetSpeed: Double) = LambdaCommand("setSpeed")
        .setStart {
            power = (targetSpeed / SPEED_AT_MAX_POWER).coerceIn(0.0, 1.0)
            this.targetSpeed = targetSpeed
        }
        .setUpdate {
            val speedError = targetSpeed - speed
            val proportionalPower = speedError * PROPORTIONAL_GAIN
            power = ((targetSpeed / SPEED_AT_MAX_POWER) + proportionalPower).coerceIn(0.0, 1.0)
        }
                .setIsDone { abs(speed - targetSpeed) < SPEED_TOLERANCE }
        .requires(this)

}
