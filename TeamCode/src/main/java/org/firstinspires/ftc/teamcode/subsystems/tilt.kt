package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.utils.BotState

object Tilt : Subsystem {
    val servo = ServoEx("tilt")
    const val UP = 0.75
    const val DOWN = 0.25
    var position = UP

    override fun initialize() {
        if (!BotState.enabled) {
            return
        }
        servo.position = position
    }

    override fun periodic() {
        if (!BotState.enabled) {
            return
        }
        servo.position = position
    }

    fun up() = InstantCommand { position = UP }

    fun down() = InstantCommand { position = DOWN }
}
