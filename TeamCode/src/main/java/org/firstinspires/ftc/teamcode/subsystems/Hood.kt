package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object Hood : Subsystem {
  val servo = ServoEx("hood")
  private var position = 0.5
  private const val BUMP_AMOUNT = 0.25

  override fun initialize() {
    servo.position = position
  }

  override fun periodic() {}

  fun setPosition(pos: Double) = InstantCommand {
    position = pos.coerceIn(0.0, 1.0)
    servo.position = position
  }

  fun bumpUp() = InstantCommand {
    position = (position + BUMP_AMOUNT).coerceIn(0.0, 1.0)
    servo.position = position
  }

  fun bumpDown() = InstantCommand {
    position = (position - BUMP_AMOUNT).coerceIn(0.0, 1.0)
    servo.position = position
  }
}
