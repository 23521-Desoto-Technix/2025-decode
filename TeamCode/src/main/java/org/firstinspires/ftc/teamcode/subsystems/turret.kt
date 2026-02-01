package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.AnalogInput
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.ServoEx

object Turret : Subsystem {
  val left = ServoEx("turretLeft")
  val right = ServoEx("turretRight")
  lateinit var encoder: AnalogInput
  val DEADZONE = 60.deg
  val MAGIC_NUMBER = 1.04
  const val RIGHT_OFFSET = 0.028

  private var lastVoltage: Double = 0.0
  private var lastTimeNs: Long = 0L

  var currentAngle: Angle = 0.0.deg
    private set

  override fun initialize() {
    encoder = ActiveOpMode.hardwareMap.analogInput["turretEncoder"]
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

    currentAngle = positionDeg.deg

    ActiveOpMode.telemetry.addData("Turret Current Position (V)", currentVoltage)
    ActiveOpMode.telemetry.addData("Turret Current Position (deg)", positionDeg)
  }

  fun setTargetAngle(angle: Angle) {
    val normalizedAngle =
        angle.normalized.inDeg.coerceIn(((-180).deg + DEADZONE).inDeg, (180.deg - DEADZONE).inDeg)
    val targetPosition = ((-normalizedAngle + 177.5) / 355.0) * MAGIC_NUMBER
    left.position = targetPosition
    right.position = targetPosition + RIGHT_OFFSET
  }

  fun setRawPosition(position: Double) {
    left.position = position
    right.position = position + RIGHT_OFFSET
  }
}
