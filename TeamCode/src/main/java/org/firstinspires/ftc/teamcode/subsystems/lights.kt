package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object lights : Subsystem {
    val right = ServoEx("rightRGB")
    val left = ServoEx("leftRGB")

    override fun periodic() {
        right.position = right.position + 0.001
        left.position = left.position + 0.001
    }
}