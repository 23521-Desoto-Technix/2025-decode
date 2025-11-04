package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.abs

object Turret : Subsystem {
  val motor = MotorEx("turret").zeroed().brakeMode()
  val encoder = MotorEx("frontLeft").zeroed()
  var angle = 0.0
  var power = 0.0
  val PID = controlSystem { posPid(0.0006, 0.0, 0.000002) }
  var usingPID = false
  var usingIMU = false
  var targetAngle = 0.0
  var IMUDegrees = 0.0
  var goal = KineticState(0.0, 0.0)
  var baseAngle = 0.0

  private fun normalizeAngle(angleDeg: Double): Double {
    var normalized = angleDeg
    while (normalized > 180) {
      normalized -= 360
    }
    while (normalized < -180) {
      normalized += 360
    }
    return normalized
  }

  private fun applyTickLimits(ticks: Double): Double {
    return ticks.coerceIn(-19_000.0, 19_000.0)
  }

  private fun degreesToTicks(degrees: Double): Double {
    return degrees * (145 / 24) * 8192 / 360
  }

  private fun ticksToDegrees(ticks: Double): Double {
    return ticks / 8192 * 360 * (24 / 145.0)
  }

  private fun setGoalSafe(angleDeg: Double, updateBase: Boolean = true) {
    val normalized = normalizeAngle(angleDeg)
    val ticks = degreesToTicks(normalized)
    val limited = applyTickLimits(ticks)
    PID.goal = KineticState(limited, 0.0)
    goal = KineticState(limited, 0.0)
    targetAngle = normalized
    if (updateBase) {
      baseAngle = normalized
    }
  }

  override fun periodic() {
    if (usingIMU) {
      val adjustedAngle = baseAngle + IMUDegrees
      setGoalSafe(adjustedAngle, false)
    }
    if (usingPID) {
      motor.power =
          PID.calculate(KineticState(encoder.currentPosition.toDouble(), encoder.velocity))
    } else {
      motor.power = power
    }
    angle = ticksToDegrees(encoder.currentPosition.toDouble())
  }

  fun setTicks(targetTicks: Double) =
      LambdaCommand("setTurretAngle")
          .setStart {
            val limited = applyTickLimits(targetTicks)
            PID.goal = KineticState(limited, 0.0)
            usingPID = true
          }
          .setIsDone { abs(angle - targetTicks) < 50 }
          .requires(this)

  fun setAngle(angle: Angle, useIMU: Boolean = false) =
      LambdaCommand().setStart {
        usingIMU = useIMU
        setGoalSafe(angle.inDeg)
        usingPID = true
      }

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
