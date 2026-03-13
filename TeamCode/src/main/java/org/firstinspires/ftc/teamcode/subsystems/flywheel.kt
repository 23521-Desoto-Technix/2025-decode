package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.VoltageCompensatingMotor
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.utils.BotState

object Flywheel : Subsystem {

  private val upperShooterMotor = VoltageCompensatingMotor(MotorEx("leftShooter").brakeMode())
  private val lowerShooterMotor =
      VoltageCompensatingMotor(MotorEx("rightShooter").brakeMode().reversed())
  private val shooterEncoder = MotorEx("backRight").zeroed()
  val PID = controlSystem {
    velPid(0.02, 0.0, 0.0)
    basicFF(0.0004)
  }

  var power = 0.0

  var braking = false

  var speed = 0.0
  var targetSpeed = 0.0

  var usingPID = false
  var enabled = true

  private const val SPEED_TOLERANCE = 30

  override fun initialize() {
    if (!BotState.enabled) {
      upperShooterMotor.power = 0.0
      lowerShooterMotor.power = 0.0
    }
  }

  override fun periodic() {
    this.speed = -shooterEncoder.velocity
    if (this.braking) {
      upperShooterMotor.power = -0.5
      lowerShooterMotor.power = -0.5
      if (this.speed < 300.0) {
        this.braking = false
        this.enabled = false
      }
      return
    }
    val hardwareEnabled = enabled && BotState.enabled
    if (!hardwareEnabled) {
      upperShooterMotor.power = 0.0
      lowerShooterMotor.power = 0.0
      return
    }

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
      upperShooterMotor.power = pidOutput
      lowerShooterMotor.power = pidOutput
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
      LambdaCommand("enableShooter").setStart { enabled = true }.setIsDone { true }.requires(this)

  fun disable() =
      LambdaCommand("disableShooter").setStart { enabled = false }.setIsDone { true }.requires(this)

  fun stop(instant: Boolean = false) =
      LambdaCommand("stopShooter")
          .setStart { this.braking = true }
          .setIsDone { instant || this.speed < 300.0 }
}
