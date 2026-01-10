package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object Hood : Subsystem {
  val servo = ServoEx("hood")
  var position = 0.5
  private const val BUMP_AMOUNT = 0.05

  override fun initialize() {
    servo.position = position
  }

  override fun periodic() {
      servo.position = position
  }

  fun bumpUp() = InstantCommand {
    position = (position + BUMP_AMOUNT).coerceIn(0.0, 1.0)
  }

  fun bumpDown() = InstantCommand {
    position = (position - BUMP_AMOUNT).coerceIn(0.0, 1.0)
  }
}
