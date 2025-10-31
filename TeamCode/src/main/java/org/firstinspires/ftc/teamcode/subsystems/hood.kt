package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

object Hood : Subsystem {
  val servo = ServoEx("hood")
  var position = 0.0

  override fun periodic() {
    servo.position = position
  }

  fun setPosition(pos: Double) = LambdaCommand("setHoodPosition").setStart { position = pos }
}
