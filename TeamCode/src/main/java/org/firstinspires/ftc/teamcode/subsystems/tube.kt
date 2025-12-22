package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.nanoseconds

private enum class TubeState {
  IDLE,
  INTAKE_PHASE0,
  INTAKE_DELAY_AFTER_TOP,
  INTAKE_WAIT_MIDDLE,
  INTAKE_DELAY_AFTER_MIDDLE,
  INTAKE_WAIT_BOTTOM,
  INTAKE_DELAY_AFTER_BOTTOM,
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

  override fun initialize() {
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
    advanceStateMachine()
    ActiveOpMode.telemetry.addData("TubeState", state)
    ActiveOpMode.telemetry.addData("Top", top.state)
    ActiveOpMode.telemetry.addData("Middle", middle.state)
    ActiveOpMode.telemetry.addData("Bottom", bottom.state)
  }

  val intakeAll = InstantCommand { transitionTo(TubeState.INTAKE_PHASE0) }

  val stopAll = InstantCommand { transitionTo(TubeState.IDLE) }

  val shootAll = InstantCommand { transitionTo(TubeState.SHOOTING_WAIT_CLEAR) }

  private fun advanceStateMachine() {
    when (state) {
      TubeState.INTAKE_PHASE0 -> if (!top.state) transitionTo(TubeState.INTAKE_DELAY_AFTER_TOP)
      TubeState.INTAKE_DELAY_AFTER_TOP ->
          if (elapsedSinceStep() >= 200.milliseconds) transitionTo(TubeState.INTAKE_WAIT_MIDDLE)
      TubeState.INTAKE_WAIT_MIDDLE ->
          if (!middle.state) transitionTo(TubeState.INTAKE_DELAY_AFTER_MIDDLE)
      TubeState.INTAKE_DELAY_AFTER_MIDDLE ->
          if (elapsedSinceStep() >= 200.milliseconds) transitionTo(TubeState.INTAKE_WAIT_BOTTOM)
      TubeState.INTAKE_WAIT_BOTTOM ->
          if (!bottom.state) transitionTo(TubeState.INTAKE_DELAY_AFTER_BOTTOM)
      TubeState.INTAKE_DELAY_AFTER_BOTTOM ->
          if (elapsedSinceStep() >= 100.milliseconds) transitionTo(TubeState.IDLE)
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
    applyStateOutputs(newState)
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
      TubeState.SHOOTING_WAIT_CLEAR,
      TubeState.SHOOTING_DELAY_BEFORE_IDLE -> {
        intake.power = 1.0
        transfer.power = 1.0
        hardStop.position = 0.65
      }
    }
  }

  private fun markStepStart() {
    stepStartedAt = now()
  }

  private fun elapsedSinceStep() = (now() - stepStartedAt).nanoseconds

  private fun now() = System.nanoTime()
}
