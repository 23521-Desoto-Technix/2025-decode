package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.impl.MotorEx

@TeleOp
class MotorTest : NextFTCOpMode() {
    private lateinit var frontLeft: MotorEx
    private lateinit var frontRight: MotorEx
    private lateinit var backLeft: MotorEx
    private lateinit var backRight: MotorEx

    private companion object {
        const val MOTOR_POWER = 0.5
    }

    override fun onInit() {
        frontLeft = MotorEx("frontLeft")
        frontRight = MotorEx("frontRight")
        backLeft = MotorEx("backLeft")
        backRight = MotorEx("backRight")

        telemetry.addData("Status", "Initialized")
        telemetry.update()
    }


    override fun onUpdate() {
        var currentlyMoving = ""

        when {
            gamepad1.square -> {
                frontLeft.power = MOTOR_POWER
                frontRight.power = 0.0
                backLeft.power = 0.0
                backRight.power = 0.0
                currentlyMoving = "Front Left"
            }
            gamepad1.circle -> {
                frontLeft.power = 0.0
                frontRight.power = MOTOR_POWER
                backLeft.power = 0.0
                backRight.power = 0.0
                currentlyMoving = "Front Right"
            }
            gamepad1.cross -> {
                frontLeft.power = 0.0
                frontRight.power = 0.0
                backLeft.power = MOTOR_POWER
                backRight.power = 0.0
                currentlyMoving = "Back Left"
            }
            gamepad1.triangle -> {
                frontLeft.power = 0.0
                frontRight.power = 0.0
                backLeft.power = 0.0
                backRight.power = MOTOR_POWER
                currentlyMoving = "Back Right"
            }
            else -> {
                frontLeft.power = 0.0
                frontRight.power = 0.0
                backLeft.power = 0.0
                backRight.power = 0.0
                currentlyMoving = "None"
            }
        }

        telemetry.addData("Motor Power", String.format("%.2f", MOTOR_POWER))
        telemetry.addData("Currently Moving", currentlyMoving)
        telemetry.addData("Front Left Power", String.format("%.2f", frontLeft.power))
        telemetry.addData("Front Right Power", String.format("%.2f", frontRight.power))
        telemetry.addData("Back Left Power", String.format("%.2f", backLeft.power))
        telemetry.addData("Back Right Power", String.format("%.2f", backRight.power))
        telemetry.addData("Controls", "Square=FL, Circle=FR, Cross=BL, Triangle=BR")
        telemetry.update()
    }

    override fun onStop() {
        frontLeft.power = 0.0
        frontRight.power = 0.0
        backLeft.power = 0.0
        backRight.power = 0.0
    }
}