package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.abs

object Shooter : Subsystem {
    private val upperShooterMotor = MotorEx("leftShooter").brakeMode().reversed()
    private val lowerShooterMotor = MotorEx("rightShooter").brakeMode()
    private val shooterEncoder = MotorEx("backLeft")

    var power = 0.0

    var speed = 0.0
    var targetSpeed = 0.0

    private const val SPEED_AT_MAX_POWER = 2_500
    private const val PROPORTIONAL_GAIN = 0.0001
    private const val SPEED_TOLERANCE = 50

    override fun periodic() {
        speed = shooterEncoder.velocity
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
        .setIsDone { (abs(targetSpeed - speed) <= SPEED_TOLERANCE) || power >= 1.0 }
        .requires(this)
}
