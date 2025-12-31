package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.CRServoEx

object Turret : Subsystem {
  val left = CRServoEx("turretLeft")
  val right = CRServoEx("turretRight")
  val encoder = ActiveOpMode.hardwareMap.analogInput["turretEncoder"]
  val pid = controlSystem { posPid(0.3, 0.0, 0.0) }

  private val DEADZONE = 45.deg
  private val MIN_ANGLE = -180.deg + DEADZONE
  private val MAX_ANGLE = 180.deg - DEADZONE

  private var lastVoltage: Double = 0.0
  private var lastTimeNs: Long = 0L

  override fun initialize() {
    lastVoltage = encoder.voltage
    lastTimeNs = System.nanoTime()
  }

  override fun periodic() {
    val now = System.nanoTime()
    val currentVoltage = encoder.voltage
    val dtSeconds = (now - lastTimeNs) / 1e9
    val velocityVoltsPerSec =
        if (dtSeconds > 0.0) (currentVoltage - lastVoltage) / dtSeconds else 0.0

    lastVoltage = currentVoltage
    lastTimeNs = now

    val scale = 360.0 / 3.3
    val positionDeg = currentVoltage * scale - 180.0
    val velocityDegPerSec = velocityVoltsPerSec * scale

    val power = pid.calculate(KineticState(positionDeg, velocityDegPerSec))
    left.power = power
    right.power = power
  }

  fun setTargetAngle(angle: Angle) {
    val normalizedAngle = angle.wrapped

    val clampedAngle =
        when {
          normalizedAngle >= MIN_ANGLE && normalizedAngle <= MAX_ANGLE -> normalizedAngle
          normalizedAngle > MAX_ANGLE -> MAX_ANGLE
          else -> MIN_ANGLE
        }
    pid.goal = KineticState(clampedAngle.inDeg, 0.0)
  }
}
