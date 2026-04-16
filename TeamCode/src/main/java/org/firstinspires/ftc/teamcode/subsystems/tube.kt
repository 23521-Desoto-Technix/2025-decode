package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.utils.BotState
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.nanoseconds

private enum class TubeState {
    IDLE,
    INTAKE_WAIT_TOP,
    INTAKE_DELAY_AFTER_TOP,
    INTAKE_WAIT_MIDDLE,
    INTAKE_DELAY_AFTER_MIDDLE,
    INTAKE_WAIT_BOTTOM,
    INTAKE_DELAY_AFTER_BOTTOM,
    SHOOTING_HARDSTOP_SETTLE,
    SHOOTING_WAIT_CLEAR,
    SHOOTING_DELAY_BEFORE_IDLE,
}

object Tube : Subsystem {
    private val intakeTopDelay = 100.milliseconds
    private val intakeMiddleDelay = 200.milliseconds
    private val intakeBottomDelay = 150.milliseconds

    val intake = MotorEx("intake").reversed()
    val transfer = MotorEx("transfer")
    val hardStop = ServoEx("hardStop")
    lateinit var top: DigitalChannel
    lateinit var middleA: DigitalChannel
    lateinit var middleB: DigitalChannel
    lateinit var bottom: DigitalChannel

    private var state = TubeState.IDLE
    private var stepStartedAt = now()
    private var shootSpeed = 1.0
    private var waitForAllStartedAt: Long? = null

    override fun initialize() {
        if (!BotState.enabled) {
            intake.power = 0.0
            transfer.power = 0.0
            hardStop.position = 0.9
            return
        }

        top = ActiveOpMode.hardwareMap.digitalChannel["top"]
        middleA = ActiveOpMode.hardwareMap.digitalChannel["middleA"]
        middleB = ActiveOpMode.hardwareMap.digitalChannel["middleB"]
        bottom = ActiveOpMode.hardwareMap.digitalChannel["bottom"]

        top.mode = DigitalChannel.Mode.INPUT
        middleA.mode = DigitalChannel.Mode.INPUT
        middleB.mode = DigitalChannel.Mode.INPUT
        bottom.mode = DigitalChannel.Mode.INPUT

        hardStop.position = 0.65
        hardStop.position = 0.9
        applyStateOutputs(state)
    }

    override fun periodic() {
        if (!BotState.enabled) {
            intake.power = 0.0
            transfer.power = 0.0
            hardStop.position = 0.9
            return
        }

        advanceStateMachine()
    }

    val intakeAll = InstantCommand {
        if (BotState.enabled) {
            transitionTo(TubeState.INTAKE_WAIT_TOP)
        }
    }

    fun isFull() = !top.state && (!middleA.state || !middleB.state) && !bottom.state

    val stopAll = InstantCommand { transitionTo(TubeState.IDLE) }

    fun shootAll(speed: Double = 1.0) = InstantCommand {
        if (!BotState.enabled) return@InstantCommand

        shootSpeed = speed
        transitionTo(TubeState.SHOOTING_HARDSTOP_SETTLE)
    }

    fun waitForAll(d: Duration? = null) =
        LambdaCommand()
            .setStart { waitForAllStartedAt = now() }
            .setIsDone {
                if (d == null) {
                    this.state == TubeState.INTAKE_DELAY_AFTER_BOTTOM
                } else {
                    this.state == TubeState.INTAKE_DELAY_AFTER_BOTTOM || elapsedSinceWaitStart() >= d
                }
            }

    fun jiggle() {
        hardStop.position = 0.65
        hardStop.position = 0.9
    }

    private fun advanceStateMachine() {
        when (state) {
            TubeState.INTAKE_WAIT_TOP -> {
                if (!top.state) {
                    transitionTo(TubeState.INTAKE_DELAY_AFTER_TOP)
                }
            }

            TubeState.INTAKE_DELAY_AFTER_TOP -> {
                if (elapsedSinceStep() >= intakeTopDelay) {
                    transitionTo(TubeState.INTAKE_WAIT_MIDDLE)
                }
            }

            TubeState.INTAKE_WAIT_MIDDLE -> {
                if (!middleA.state || !middleB.state) {
                    transitionTo(TubeState.INTAKE_DELAY_AFTER_MIDDLE)
                }
            }

            TubeState.INTAKE_DELAY_AFTER_MIDDLE -> {
                if (elapsedSinceStep() >= intakeMiddleDelay) {
                    transitionTo(TubeState.INTAKE_WAIT_BOTTOM)
                }
            }

            TubeState.INTAKE_WAIT_BOTTOM -> {
                if (!bottom.state) {
                    transitionTo(TubeState.INTAKE_DELAY_AFTER_BOTTOM)
                }
            }

            TubeState.INTAKE_DELAY_AFTER_BOTTOM -> {
                if (elapsedSinceStep() >= intakeBottomDelay) {
                    transitionTo(TubeState.IDLE)
                    ActiveOpMode.gamepad1.rumbleBlips(3)
                    ActiveOpMode.gamepad2.rumbleBlips(3)
                }
            }

            TubeState.SHOOTING_HARDSTOP_SETTLE -> {
                if (elapsedSinceStep() >= 100.milliseconds) {
                    transitionTo(TubeState.SHOOTING_WAIT_CLEAR)
                }
            }

            TubeState.SHOOTING_WAIT_CLEAR -> {
                if (top.state && (middleA.state || middleB.state) && bottom.state) {
                    transitionTo(TubeState.SHOOTING_DELAY_BEFORE_IDLE)
                }
            }

            TubeState.SHOOTING_DELAY_BEFORE_IDLE -> {
                if (elapsedSinceStep() >= 500.milliseconds) {
                    transitionTo(TubeState.IDLE)
                }
            }

            else -> {}
        }
    }

    private fun transitionTo(newState: TubeState) {
        if (state == newState) return

        state = newState
        markStepStart()
        applyStateOutputs(newState)
    }

    private fun applyStateOutputs(targetState: TubeState) {
        when (targetState) {
            TubeState.IDLE -> {
                intake.power = 0.0
                transfer.power = 0.0
                hardStop.position = 0.9
            }

            TubeState.INTAKE_WAIT_TOP,
            TubeState.INTAKE_DELAY_AFTER_TOP -> {
                intake.power = 1.0
                transfer.power = 1.0
                hardStop.position = 0.9
            }

            TubeState.INTAKE_WAIT_MIDDLE,
            TubeState.INTAKE_DELAY_AFTER_MIDDLE,
            TubeState.INTAKE_WAIT_BOTTOM,
            TubeState.INTAKE_DELAY_AFTER_BOTTOM -> {
                intake.power = 1.0
                transfer.power = 0.0
                hardStop.position = 0.9
            }

            TubeState.SHOOTING_HARDSTOP_SETTLE -> {
                intake.power = 0.0
                transfer.power = 0.0
                hardStop.position = 0.65
            }

            TubeState.SHOOTING_WAIT_CLEAR,
            TubeState.SHOOTING_DELAY_BEFORE_IDLE -> {
                intake.power = shootSpeed
                transfer.power = shootSpeed * 0.9
                hardStop.position = 0.65
            }
        }
    }

    private fun markStepStart() {
        stepStartedAt = now()
    }

    private fun elapsedSinceStep() = (now() - stepStartedAt).nanoseconds

    private fun elapsedSinceWaitStart(): Duration {
        return if (waitForAllStartedAt != null) {
            (now() - waitForAllStartedAt!!).nanoseconds
        } else {
            0.nanoseconds
        }
    }

    private fun now() = System.nanoTime()
}
