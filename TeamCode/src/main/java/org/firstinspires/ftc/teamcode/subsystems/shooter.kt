package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.VoltageCompensatingMotor
import kotlin.math.abs

object Shooter : Subsystem {

  private val upperShooterMotor = VoltageCompensatingMotor(MotorEx("leftShooter").brakeMode())
  private val lowerShooterMotor =
      VoltageCompensatingMotor(MotorEx("rightShooter").brakeMode().reversed())
  private val shooterEncoder = MotorEx("backLeft")
  val PID = controlSystem {
    velPid(0.01, 0.0, 0.0)
    basicFF(0.0004)
  }

  var power = 0.0

  var speed = 0.0
  var targetSpeed = 0.0

  var usingPID = false
  var enabled = true

  private const val SPEED_TOLERANCE = 100

  override fun periodic() {
    if (!enabled) {
      upperShooterMotor.power = 0.0
      lowerShooterMotor.power = 0.0
      return
    }


    this.speed = -shooterEncoder.velocity
    if (this.usingPID) {
      PID.goal = KineticState(Double.POSITIVE_INFINITY, this.targetSpeed, 0.0)
      val pidOutput =
          PID.calculate(
              KineticState(
                  Double.POSITIVE_INFINITY,
                  this.speed,
                  shooterEncoder.state.acceleration,
              )
          )
      upperShooterMotor.power = pidOutput.coerceIn(0.0, 1.0)
      lowerShooterMotor.power = pidOutput.coerceIn(0.0, 1.0)
    } else {
      upperShooterMotor.power = power
      lowerShooterMotor.power = power
    }
  }

  fun setPower(power: Double) =
      LambdaCommand("setPower")
          .setStart {
            this.usingPID = false
            this.power = power.coerceIn(0.0, 1.0)
          }
          .setIsDone { true }
          .requires(this)

  fun setSpeed(speed: Double) =
      LambdaCommand("setSpeed")
          .setStart {
            this.usingPID = true
            this.targetSpeed = speed.coerceAtLeast(0.0)
          }
          .setIsDone { true }
          .requires(this)

  fun waitForSpeed() =
      LambdaCommand("waitForSpeed")
          .setIsDone { abs(targetSpeed - speed) <= SPEED_TOLERANCE }
          .requires(this)

  fun enable() =
      LambdaCommand("enableShooter")
          .setStart { enabled = true }
          .setIsDone { true }
          .requires(this)

  fun disable() =
      LambdaCommand("disableShooter")
          .setStart { enabled = false }
          .setIsDone { true }
          .requires(this)
}
