package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedforward.FeedforwardElement
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.VoltageCompensatingMotor
import kotlin.math.abs

object Shooter : Subsystem {

  class ShooterFeedforward : FeedforwardElement {
    override fun calculate(reference: KineticState): Double {
      return targetSpeed * 0.0003
    }
  }

  private val upperShooterMotor = VoltageCompensatingMotor(MotorEx("leftShooter").brakeMode())
  private val lowerShooterMotor =
      VoltageCompensatingMotor(MotorEx("rightShooter").brakeMode().reversed())
  private val shooterEncoder = MotorEx("backLeft")
  val PID = controlSystem {
    velPid(0.0, 0.0, 0.0)
    feedforward(ShooterFeedforward())
  }

  var power = 0.0

  var speed = 0.0
  var targetSpeed = 0.0

  var usingPID = false

  private const val SPEED_TOLERANCE = 100

  override fun periodic() {
    this.speed = shooterEncoder.velocity
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
}
