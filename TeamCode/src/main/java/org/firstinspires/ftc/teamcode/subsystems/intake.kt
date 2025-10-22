package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object Intake : Subsystem {
    private val motor = MotorEx("intake").brakeMode().reversed() //TODO: Check direction

    var power = 0.0

    override fun periodic() {
        motor.power = power
    }

    fun setPower(power: Double) = LambdaCommand("setIntakePower")
        .setStart {
            this.power = power
        }
        .setIsDone { true }
        .requires(this)
}