package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object Feeder : Subsystem {
    val leftServo = ServoEx("leftFeeder")
    val rightServo = ServoEx("rightFeeder")

    fun feed() = LambdaCommand("feed")
        .setStart {
            leftServo.position = 0.61 //TODO TUNE THIS
            rightServo.position = 0.71 //TODO TUNE THIS
        }
        .setIsDone { true }
        .requires(this)
    fun reset() = LambdaCommand("resetFeeder")
        .setStart {
            leftServo.position = 0.9 //TODO TUNE THIS
            rightServo.position = 1.0 //TODO TUNE THIS
        }
        .setIsDone { true }
        .requires(this)
}