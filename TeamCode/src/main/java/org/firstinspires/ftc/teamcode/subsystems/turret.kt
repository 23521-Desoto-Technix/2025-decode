package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.AnalogInput
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
  lateinit var encoder: AnalogInput
  val pid = controlSystem { posPid(0.017, 0.0, 0.000107) }
  val DEADZONE = 65.deg

  private var lastVoltage: Double = 0.0
  private var lastTimeNs: Long = 0L

  override fun initialize() {
    encoder = ActiveOpMode.hardwareMap.analogInput["turretEncoder"]
    lastVoltage = encoder.voltage
    lastTimeNs = System.nanoTime()
    pid.goal = KineticState(0.0, 0.0)
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

    val power = -pid.calculate(KineticState(positionDeg, velocityDegPerSec))
    left.power = power
    right.power = power

    val targetVolts = (pid.goal.position + 180.0) / scale
    ActiveOpMode.telemetry.addData("Turret Current Position (V)", currentVoltage)
    ActiveOpMode.telemetry.addData("Turret Target Position (V)", targetVolts)
    ActiveOpMode.telemetry.addData("Turret Current Position (deg)", positionDeg)
    ActiveOpMode.telemetry.addData("Turret Target Position (deg)", pid.goal.position)
  }

  fun setTargetAngle(angle: Angle) {
    val normalizedAngle =
        angle.normalized.inDeg.coerceIn(((-180).deg + DEADZONE).inDeg, (180.deg - DEADZONE).inDeg)
    pid.goal = KineticState(normalizedAngle, 0.0)
  }
}
