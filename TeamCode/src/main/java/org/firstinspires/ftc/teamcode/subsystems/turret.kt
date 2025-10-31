package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object Turret : Subsystem {
    val motor = MotorEx("turret").zeroed().brakeMode()
    val encoder = MotorEx("frontLeft")
    var angle = 0.0
    var power = 0.0
    var previousError = 0.0
    var lastTime = System.currentTimeMillis()

    override fun periodic() {
        motor.power = power
        angle = encoder.currentPosition.toDouble()
    }

    fun setPower(power: Double) = LambdaCommand("setTurretPower")
        .setStart {
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
    fun cameraTrackPower(targetAngle: Double) = LambdaCommand("turretCameraTrackPower")
    .setStart {
        val kP = 0.002
        val kD = 0.0002
        val error = targetAngle
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