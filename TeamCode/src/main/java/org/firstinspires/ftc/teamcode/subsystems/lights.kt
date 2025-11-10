package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object Lights : Subsystem {
    val right = ServoEx("rightRGB")
    val left = ServoEx("leftRGB")
    var state: LightsState = LightsState.ALLIANCE_UNKNOWN

    private var blinkStartTime = 0L
    private val BLINK_CYCLE_TIME = 750L
    private val ALLIANCE_UNKNOWN_ON = 0.388
    private val ALLIANCE_UNKNOWN_OFF = 0.0

    override fun periodic() {
        if (state == LightsState.ALLIANCE_UNKNOWN && blinkStartTime == 0L) {
            blinkStartTime = System.currentTimeMillis()
        }

        when (state) {
            LightsState.OFF -> {
                right.position = 0.0
                left.position = 0.0
            }
            LightsState.ALLIANCE_RED -> {
                right.position = 0.285
                left.position = 0.285
            }
            LightsState.ALLIANCE_BLUE -> {
                right.position = 0.611
                left.position = 0.611
            }
            LightsState.ALLIANCE_UNKNOWN -> {
                val currentTime = System.currentTimeMillis()
                val elapsedTime = currentTime - blinkStartTime
                val positionInCycle = elapsedTime % BLINK_CYCLE_TIME
                val position = if (positionInCycle < BLINK_CYCLE_TIME / 2) {
                    ALLIANCE_UNKNOWN_ON
                } else {
                    ALLIANCE_UNKNOWN_OFF
                }
                right.position = position
                left.position = position
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
            LightsState.ARTIFACT_GREEN -> {
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
            LightsState.ARTIFACT_PURPLE -> {
                right.position = 0.722
                left.position = 0.722
            }
            LightsState.ARTIFACT_FULL -> {
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
    ARTIFACT_GREEN,
    DEBUG_AZURE,
    DEBUG_BLUE,
    DEBUG_INDIGO,
    ARTIFACT_PURPLE,
    ARTIFACT_FULL,
}
