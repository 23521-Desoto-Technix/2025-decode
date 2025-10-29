package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object Lights : Subsystem {
    val right = ServoEx("rightRGB")
    val left = ServoEx("leftRGB")
    var state: LightsState = LightsState.OFF
        set(value) {
            val stateChanged = field != value
            field = value
            if (stateChanged) {
                animationStartTime = System.currentTimeMillis()
            }
        }
    private var lastUpdateTime: Long = System.currentTimeMillis()
    private var animationStartTime: Long = System.currentTimeMillis()
    private var colorPosition: Double = 0.28

    override fun periodic() {
        val currentTime = System.currentTimeMillis()
        val elapsedSinceUpdate = currentTime - lastUpdateTime
        lastUpdateTime = currentTime
        val elapsedSinceStart = currentTime - animationStartTime

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
            LightsState.IDLE -> {
                val cycleDuration = 4000L
                val cycleProgress = (elapsedSinceStart % cycleDuration).toDouble() / cycleDuration
                colorPosition = 0.28 + (cycleProgress * 0.43)
                right.position = colorPosition
                left.position = colorPosition
            }
            LightsState.DEBUG_RED -> {
                right.position = 0.277
                left.position = 0.277
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
                right.position = 0.611
                left.position = 0.611
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
            LightsState.DEBUG_ALTERNATING_RED_BLUE -> {
                val cycleDuration = 1000L
                val isFirstHalf = (elapsedSinceStart % cycleDuration) < 500
                if (isFirstHalf) {
                    right.position = 0.277
                    left.position = 0.277
                } else {
                    right.position = 0.611
                    left.position = 0.611
                }
            }
            LightsState.DEBUG_ALTERNATING_RIGHT_LEFT -> {
                val cycleDuration = 1000L
                val isFirstHalf = (elapsedSinceStart % cycleDuration) < 500
                if (isFirstHalf) {
                    right.position = 0.277
                    left.position = 1.0
                } else {
                    right.position = 1.0
                    left.position = 0.277
                }
            }
        }
    }

    fun setState(newState: LightsState) : LambdaCommand = LambdaCommand("setLightsState")
        .setStart {
            state = newState
        }
        .setIsDone { true }
        .requires(this)
}

enum class LightsState {
    OFF,
    ALLIANCE_RED,
    ALLIANCE_BLUE,
    ALLIANCE_UNKNOWN,
    IDLE,
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
    DEBUG_ALTERNATING_RED_BLUE,
    DEBUG_ALTERNATING_RIGHT_LEFT,
}
