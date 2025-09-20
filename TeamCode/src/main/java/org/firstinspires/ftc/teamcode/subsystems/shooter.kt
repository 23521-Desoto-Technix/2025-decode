package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object shooter : Subsystem {
    private val upperShooterMotor = MotorEx("upperShooter").brakeMode().reversed()
    private val lowerShooterMotor = MotorEx("lowerShooter").brakeMode()

    private var power = 0.0

    var speed = 0.0

    private const val MIN_SPEED = 2_600
    private const val STOP_SPEED = 150

    override fun periodic() {
        speed = upperShooterMotor.state.velocity
        upperShooterMotor.power = power
        lowerShooterMotor.power = power
    }

    val spinUp = LambdaCommand("spinUp")
        .setStart {
            power = 1.0
        }
        .setIsDone { speed >= MIN_SPEED }
    val spinDown = LambdaCommand("spinDown")
        .setStart {
            power = 0.0
        }
        .setIsDone { speed <= STOP_SPEED }
}