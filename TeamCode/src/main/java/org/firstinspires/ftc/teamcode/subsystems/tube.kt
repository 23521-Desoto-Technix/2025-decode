package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.nanoseconds
import org.firstinspires.ftc.teamcode.utils.BotState

private enum class TubeState {
  IDLE,
  INTAKE_PHASE0,
  INTAKE_DELAY_AFTER_TOP,
  INTAKE_WAIT_MIDDLE,
  INTAKE_DELAY_AFTER_MIDDLE,
  INTAKE_WAIT_BOTTOM,
  INTAKE_DELAY_AFTER_BOTTOM,
  INTAKE_FINAL_PUSH,
  SHOOTING_HARDSTOP_SETTLE,
  SHOOTING_WAIT_CLEAR,
  SHOOTING_DELAY_BEFORE_IDLE,
}

object Tube : Subsystem {
  val intake = MotorEx("intake").reversed()
  val transfer = MotorEx("transfer")
  val hardStop = ServoEx("hardStop")
  lateinit var top: DigitalChannel
  lateinit var middle: DigitalChannel
  lateinit var bottom: DigitalChannel
  private var state = TubeState.IDLE
  private var stepStartedAt = now()
  private var shootSpeed = 1.0
  private var bottomTripStartedAt: Long? = null
  private var lastBottomState = true
  private var waitForAllStartedAt: Long? = null

  override fun initialize() {
    if (!BotState.enabled) {
      intake.power = 0.0
      transfer.power = 0.0
      hardStop.position = 0.9
      return
    }
    top = ActiveOpMode.hardwareMap.digitalChannel["top"]
    middle = ActiveOpMode.hardwareMap.digitalChannel["middle"]
    bottom = ActiveOpMode.hardwareMap.digitalChannel["bottom"]
    top.mode = DigitalChannel.Mode.INPUT
    middle.mode = DigitalChannel.Mode.INPUT
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
    ActiveOpMode.telemetry.addData("TubeState", state)
    ActiveOpMode.telemetry.addData("Top", top.state)
    ActiveOpMode.telemetry.addData("Middle", middle.state)
    ActiveOpMode.telemetry.addData("Bottom", bottom.state)
  }

  val intakeAll = InstantCommand {
    if (BotState.enabled) {
      transitionTo(TubeState.INTAKE_PHASE0)
    }
  }

  fun isFull() = !top.state && !middle.state && !bottom.state

  val stopAll = InstantCommand { transitionTo(TubeState.IDLE) }

  fun shootAll(speed: Double = 1.0) = InstantCommand {
    if (!BotState.enabled) {
      return@InstantCommand
    }
    shootSpeed = speed
    transitionTo(TubeState.SHOOTING_HARDSTOP_SETTLE)
  }

  fun waitForAll(d: Duration? = null) =
      LambdaCommand()
          .setStart { waitForAllStartedAt = now() }
          .setIsDone {
            if (d == null) {
              this.state == TubeState.IDLE
            } else {
              this.state == TubeState.IDLE || elapsedSinceWaitStart() >= d
            }
          }

  private fun advanceStateMachine() {
    updateBottomTripTimer()

    when (state) {
      TubeState.INTAKE_PHASE0 -> {
        if (isBottomContinuouslyTripped(1000.milliseconds)) {
          transitionTo(TubeState.INTAKE_FINAL_PUSH)
        } else if (!top.state) {
          transitionTo(TubeState.INTAKE_DELAY_AFTER_TOP)
        }
      }
      TubeState.INTAKE_DELAY_AFTER_TOP -> {
        if (isBottomContinuouslyTripped(1000.milliseconds)) {
          transitionTo(TubeState.INTAKE_FINAL_PUSH)
        } else if (elapsedSinceStep() >= 300.milliseconds) {
          transitionTo(TubeState.INTAKE_WAIT_MIDDLE)
        }
      }
      TubeState.INTAKE_WAIT_MIDDLE -> {
        if (isBottomContinuouslyTripped(1000.milliseconds)) {
          transitionTo(TubeState.INTAKE_FINAL_PUSH)
        } else if (!middle.state) {
          transitionTo(TubeState.INTAKE_DELAY_AFTER_MIDDLE)
        }
      }
      TubeState.INTAKE_DELAY_AFTER_MIDDLE -> {
        if (isBottomContinuouslyTripped(1000.milliseconds)) {
          transitionTo(TubeState.INTAKE_FINAL_PUSH)
        } else if (elapsedSinceStep() >= 200.milliseconds) {
          transitionTo(TubeState.INTAKE_WAIT_BOTTOM)
        }
      }
      TubeState.INTAKE_WAIT_BOTTOM -> {
        if (isBottomContinuouslyTripped(1000.milliseconds)) {
          transitionTo(TubeState.INTAKE_FINAL_PUSH)
        } else if (!bottom.state) {
          transitionTo(TubeState.INTAKE_DELAY_AFTER_BOTTOM)
        }
      }
      TubeState.INTAKE_DELAY_AFTER_BOTTOM -> {
        if (isBottomContinuouslyTripped(1000.milliseconds)) {
          transitionTo(TubeState.INTAKE_FINAL_PUSH)
        } else if (elapsedSinceStep() >= 50.milliseconds) {
          transitionTo(TubeState.INTAKE_FINAL_PUSH)
        }
      }
      TubeState.INTAKE_FINAL_PUSH ->
          if (elapsedSinceStep() >= 50.milliseconds) {
            transitionTo(TubeState.IDLE)
            ActiveOpMode.gamepad1.rumbleBlips(3)
            ActiveOpMode.gamepad2.rumbleBlips(3)
          }
      TubeState.SHOOTING_HARDSTOP_SETTLE ->
          if (elapsedSinceStep() >= 200.milliseconds) transitionTo(TubeState.SHOOTING_WAIT_CLEAR)
      TubeState.SHOOTING_WAIT_CLEAR ->
          if (top.state && middle.state && bottom.state) {
            transitionTo(TubeState.SHOOTING_DELAY_BEFORE_IDLE)
          }
      TubeState.SHOOTING_DELAY_BEFORE_IDLE ->
          if (elapsedSinceStep() >= 500.milliseconds) transitionTo(TubeState.IDLE)
      else -> {}
    }
  }

  private fun transitionTo(newState: TubeState) {
    if (state == newState) return
    state = newState
    markStepStart()
    resetBottomTripTimer()
    applyStateOutputs(newState)
  }

  private fun updateBottomTripTimer() {
    val currentBottomTripped = !bottom.state

    if (currentBottomTripped && !lastBottomState) {
      bottomTripStartedAt = now()
    } else if (!currentBottomTripped && lastBottomState) {
      bottomTripStartedAt = null
    }

    lastBottomState = currentBottomTripped
  }

  private fun isBottomContinuouslyTripped(duration: kotlin.time.Duration): Boolean {
    if (bottomTripStartedAt == null) return false
    return (now() - bottomTripStartedAt!!) >= duration.inWholeNanoseconds
  }

  private fun resetBottomTripTimer() {
    bottomTripStartedAt = null
  }

  private fun applyStateOutputs(targetState: TubeState) {
    when (targetState) {
      TubeState.IDLE -> {
        intake.power = 0.0
        transfer.power = 0.0
        hardStop.position = 0.9
      }
      TubeState.INTAKE_PHASE0 -> {
        intake.power = 1.0
        transfer.power = 1.0
        hardStop.position = 0.9
      }
      TubeState.INTAKE_DELAY_AFTER_TOP,
      TubeState.INTAKE_WAIT_MIDDLE,
      TubeState.INTAKE_DELAY_AFTER_MIDDLE,
      TubeState.INTAKE_WAIT_BOTTOM,
      TubeState.INTAKE_DELAY_AFTER_BOTTOM -> {
        intake.power = 1.0
        transfer.power = 0.0
        hardStop.position = 0.9
      }
      TubeState.INTAKE_FINAL_PUSH -> {
        intake.power = 1.0
        transfer.power = 1.0
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
        transfer.power = shootSpeed
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
