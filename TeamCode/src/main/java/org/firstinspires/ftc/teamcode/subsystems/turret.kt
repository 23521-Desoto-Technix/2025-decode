package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.abs

object Turret : Subsystem {
  val motor = MotorEx("turret").zeroed().brakeMode()
  val encoder = MotorEx("frontLeft")
  var angle = 0.0
  var power = 0.0
  var previousError = 0.0
  var lastTime = System.currentTimeMillis()
  val PID = controlSystem { posPid(0.0015, 0.0, 0.0) }
  var usingPID = false

  override fun periodic() {
    if (usingPID) {
      motor.power =
          PID.calculate(KineticState(-encoder.currentPosition.toDouble(), -encoder.velocity))
    } else {
      motor.power = power
    }
    angle = encoder.currentPosition.toDouble()
  }

  fun setAngle(targetAngle: Double) =
      LambdaCommand("setTurretAngle")
          .setStart {
            usingPID = true
            PID.goal = KineticState(targetAngle.coerceIn(-19_000.0, 19_000.0), 0.0)
          }
          .setIsDone { abs(angle - targetAngle) < 50 }
          .requires(this)

  fun setPower(power: Double) =
      LambdaCommand("setTurretPower")
          .setStart {
            usingPID = false
            if (angle >= 19_000) {
              this.power = 0.2
            } else if (angle <= -19_000) {
              this.power = -0.2
            } else {
              this.power = power
            }
          }
          .setIsDone { true }
          .requires(this)
}
