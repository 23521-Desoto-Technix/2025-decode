package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.delays.WaitUntil
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx

object Tube : Subsystem {
  val intake = MotorEx("intake")
  val transfer = MotorEx("transfer")
  val top = ActiveOpMode.hardwareMap.digitalChannel["top"]
  val middle = ActiveOpMode.hardwareMap.digitalChannel["middle"]
  val bottom = ActiveOpMode.hardwareMap.digitalChannel["bottom"]

  init {
    top.mode = com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT
    middle.mode = com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT
    bottom.mode = com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT
  }

  val intakeAll =
      ParallelGroup(
          InstantCommand {
            intake.power = 1.0
            transfer.power = 1.0
          },
          WaitUntil { top.state },
          InstantCommand { transfer.power = 0.0 },
          WaitUntil { top.state && middle.state && bottom.state },
          InstantCommand { intake.power = 0.0 },
      )
}
