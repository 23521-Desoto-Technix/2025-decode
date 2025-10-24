package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object Lights : Subsystem {
    val right = ServoEx("rightRGB")
    val left = ServoEx("leftRGB")
    var state: LightsState = LightsState.OFF

    override fun periodic() {
        if (state == LightsState.OFF) {
            right.position = 0.0
            left.position = 0.0
            return
        }
        if (state == LightsState.ALLIANCE_RED) {
            right.position = 0.28
            left.position = 0.28
            return
        }
        if (state == LightsState.ALLIANCE_BLUE) {
            right.position = 0.63
            left.position = 0.63
            return
        }
        if (state == LightsState.ALLIANCE_UNKNOWN) {
            right.position = 0.72
            left.position = 0.72
            return
        }
        if (state == LightsState.IDLE) {
            right.position += 0.02
            left.position += 0.02
            if (right.position > 0.71) {
                right.position = 0.28
            }
            if (left.position > 0.71) {
                left.position = 0.28
            }
            if (right.position < 0.28) {
                right.position = 0.28
            }
            if (left.position < 0.28) {
                left.position = 0.28
            }
        }
    }
}

enum class LightsState {
    OFF,
    ALLIANCE_RED,
    ALLIANCE_BLUE,
    ALLIANCE_UNKNOWN,
    IDLE,
}
