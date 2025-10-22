package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx

object Indexer : Subsystem {
    var power = 0.0
    val servo = CRServoEx("indexer")

    override fun periodic() {
        servo.power = power
    }

    fun setPower(power: Double) = LambdaCommand("setIndexerPower")
        .setStart {
            this.power = power
        }
        .setIsDone { true }
        .requires(this)
}