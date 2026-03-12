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

    private var lastVoltage: Double = 0.0
    private var lastTimeNs: Long = 0L

    const val OFFSET = 0.015

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
    }

    fun setTargetAngle(angle: Angle) {
        val normalizedAngle = angle.normalized.inDeg
        val targetPosition = 0.5 - normalizedAngle * 0.0031666667
        left.position = targetPosition + OFFSET
        right.position = targetPosition - OFFSET
    }

    fun setRawPosition(position: Double) {
        left.position = position + OFFSET
        right.position = position - OFFSET
    }
}
