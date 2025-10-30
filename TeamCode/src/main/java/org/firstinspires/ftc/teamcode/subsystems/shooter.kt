package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.abs

object Shooter : Subsystem {
  private val upperShooterMotor = MotorEx("leftShooter").brakeMode().reversed()
  private val lowerShooterMotor = MotorEx("rightShooter").brakeMode()
  private val shooterEncoder = MotorEx("backLeft")

  var power = 0.0

  var speed = 0.0
  var targetSpeed = 0.0

  var usingPID = true

  private const val SPEED_AT_MAX_POWER = 2_600
  private const val PROPORTIONAL_GAIN = 0.0005
  private const val SPEED_TOLERANCE = 100

  override fun periodic() {
    this.speed = shooterEncoder.velocity
    val speedError = targetSpeed - speed
    val proportionalPower = speedError * PROPORTIONAL_GAIN
    if (usingPID) {
      this.power = ((targetSpeed / SPEED_AT_MAX_POWER) + proportionalPower).coerceIn(0.0, 1.0)
    }

    upperShooterMotor.power = power
    lowerShooterMotor.power = power
  }

  fun setSpeed(targetSpeed: Double) =
      LambdaCommand("setSpeed")
          .setStart {
            this.usingPID = true
            this.power = (targetSpeed / SPEED_AT_MAX_POWER).coerceIn(0.0, 1.0)
            this.targetSpeed = targetSpeed
          }
          .setIsDone { true }
          .requires(this)

  fun setPower(power: Double) =
      LambdaCommand("setPower")
          .setStart {
            this.usingPID = false
            this.power = power.coerceIn(0.0, 1.0)
          }
          .setIsDone { true }
          .requires(this)

  fun waitForSpeed() =
      LambdaCommand("waitForSpeed")
          .setIsDone { abs(targetSpeed - speed) <= SPEED_TOLERANCE }
          .requires(this)
}
