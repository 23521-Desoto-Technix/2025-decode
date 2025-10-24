package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object Turret : Subsystem {
    val motor = MotorEx("turret").zeroed().brakeMode()
    val encoder = MotorEx("frontLeft")
    var angle = 0.0
    var power = 0.0

    override fun periodic() {
        motor.power = power
        angle = encoder.currentPosition.toDouble()
    }

    fun setPower(power: Double) = LambdaCommand("setTurretPower")
        .setStart {
            if (angle >= 19_000) {
                this.power = 0.2
            } else if (angle <= -19_000) {
                this.power = -0.2
            } else {
                this.power = power
            }
    }
        .setIsDone { true }
        .requires(this)
}