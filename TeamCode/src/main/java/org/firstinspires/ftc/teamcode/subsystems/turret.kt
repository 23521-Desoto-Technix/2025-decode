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
  val PID = controlSystem { posPid(0.0002, 0.0, 0.000002) }
  var usingPID = false
  var usingIMU = false
  var targetAngle = 0.0
  var IMUDegrees = 0.0
  var goal = KineticState(0.0, 0.0)
  var baseAngle = 0.0
  var previousError = 0.0
  var lastTime = System.currentTimeMillis()

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
    return degrees * (145.0 / 24.0) * 8192 / 360
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
          .setIsDone { abs(encoder.currentPosition.toDouble() - targetTicks) < 50 }
          .requires(this)

  fun setAngle(angle: Angle, useIMU: Boolean = false) =
      LambdaCommand("setTurretAngle")
          .setStart {
            usingIMU = useIMU
            setGoalSafe(angle.inDeg)
            usingPID = true
          }
          .setIsDone { true }
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

  fun cameraTrackPower(cameraError: Double, gamepadComp: Double = 0.0) =
      LambdaCommand("turretCameraTrackPower")
          .setStart {
            usingPID = false

            var kP = 0.0015
            val kD = 0.00001
            val error = cameraError + (gamepadComp * 150.0)
            val currentTime = System.currentTimeMillis()
            val dt = (currentTime - lastTime) / 1000.0
            val errorRate = if (dt > 0) (error - previousError) / dt else 0.0

            if (angle >= 19_000) {
              this.power = 0.2
            } else if (angle <= -19_000) {
              this.power = -0.2
            } else {
              this.power = ((error * kP) + (errorRate * kD)).coerceIn(-1.0, 1.0)
            }

            previousError = error
            lastTime = currentTime
          }
          .setIsDone { true }
          .requires(this)
}
