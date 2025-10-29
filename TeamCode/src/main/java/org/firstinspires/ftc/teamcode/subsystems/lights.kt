package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object Lights : Subsystem {
    val right = ServoEx("rightRGB")
    val left = ServoEx("leftRGB")
    var state: LightsState = LightsState.ALLIANCE_UNKNOWN

    override fun periodic() {
        when (state) {
            LightsState.OFF -> {
                right.position = 0.0
                left.position = 0.0
            }
            LightsState.ALLIANCE_RED -> {
                right.position = 0.277
                left.position = 0.277
            }
            LightsState.ALLIANCE_BLUE -> {
                right.position = 0.611
                left.position = 0.611
            }
            LightsState.ALLIANCE_UNKNOWN -> {
                right.position = 0.388
                left.position = 0.388
            }
            LightsState.DEBUG_RED -> {
                right.position = 0.305
                left.position = 0.305
            }
            LightsState.DEBUG_ORANGE -> {
                right.position = 0.333
                left.position = 0.333
            }
            LightsState.DEBUG_YELLOW -> {
                right.position = 0.388
                left.position = 0.388
            }
            LightsState.DEBUG_SAGE -> {
                right.position = 0.444
                left.position = 0.444
            }
            LightsState.DEBUG_GREEN -> {
                right.position = 0.500
                left.position = 0.500
            }
            LightsState.DEBUG_AZURE -> {
                right.position = 0.555
                left.position = 0.555
            }
            LightsState.DEBUG_BLUE -> {
                right.position = 0.583
                left.position = 0.583
            }
            LightsState.DEBUG_INDIGO -> {
                right.position = 0.666
                left.position = 0.666
            }
            LightsState.DEBUG_PURPLE -> {
                right.position = 0.722
                left.position = 0.722
            }
            LightsState.DEBUG_WHITE -> {
                right.position = 1.0
                left.position = 1.0
            }
        }
    }

}

enum class LightsState {
    OFF,
    ALLIANCE_RED,
    ALLIANCE_BLUE,
    ALLIANCE_UNKNOWN,
    DEBUG_RED,
    DEBUG_ORANGE,
    DEBUG_YELLOW,
    DEBUG_SAGE,
    DEBUG_GREEN,
    DEBUG_AZURE,
    DEBUG_BLUE,
    DEBUG_INDIGO,
    DEBUG_PURPLE,
    DEBUG_WHITE,
}
