package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import kotlin.time.Duration.Companion.milliseconds

object Tube : Subsystem {
  val intake = MotorEx("intake").reversed()
  val transfer = MotorEx("transfer")
  val hardStop = ServoEx("hardStop")
  lateinit var top: DigitalChannel
  lateinit var middle: DigitalChannel
  lateinit var bottom: DigitalChannel

  override fun initialize() {
    top = ActiveOpMode.hardwareMap.digitalChannel["top"]
    middle = ActiveOpMode.hardwareMap.digitalChannel["middle"]
    bottom = ActiveOpMode.hardwareMap.digitalChannel["bottom"]
    top.mode = DigitalChannel.Mode.INPUT
    middle.mode = DigitalChannel.Mode.INPUT
    bottom.mode = DigitalChannel.Mode.INPUT
    hardStop.position = 0.65
    hardStop.position = 0.9
  }

  override fun periodic() {
    ActiveOpMode.telemetry.addData("Top", top.state)
    ActiveOpMode.telemetry.addData("Middle", middle.state)
    ActiveOpMode.telemetry.addData("Bottom", bottom.state)
  }

  val intakeAll =
      InstantCommand {
            intake.power = 1.0
            transfer.power = 1.0
            hardStop.position = 0.9
          }
          .then(WaitUntil { !top.state })
          .then(InstantCommand { transfer.power = 0.0 })
          .then(Delay(200.milliseconds))
          .then(WaitUntil { !middle.state })
          .then(Delay(200.milliseconds))
          .then(WaitUntil { !bottom.state })
          .then(InstantCommand { intake.power = 0.0 })

  val stopAll = InstantCommand {
      intake.power = 0.0
      transfer.power = 0.0
  }

  val shootAll =
      InstantCommand {
            intake.power = 1.0
            transfer.power = 1.0
            hardStop.position = 0.65
          }
          .then(WaitUntil { top.state && middle.state && bottom.state })
          .then(Delay(500.milliseconds))
          .then(
              InstantCommand {
                intake.power = 0.0
                transfer.power = 0.0
              }
          )
}
