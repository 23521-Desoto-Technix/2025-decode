package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "PTO Test")
class ptoTest : LinearOpMode() {
    override fun runOpMode() {
        val pto = hardwareMap.servo["pto"]
        val left = hardwareMap.dcMotor["left"]
        val right = hardwareMap.dcMotor["right"]
        right.direction = DcMotorSimple.Direction.REVERSE
        var power = 0.0
        waitForStart()
        while (opModeIsActive()) {
            left.power = power
            right.power = power
            if (gamepad1.leftBumperWasPressed()) {
                power += 0.1
            } else if (gamepad1.rightBumperWasPressed()) {
                power -= 0.1
            }
            if (gamepad1.a) {
                pto.position = 0.0
            } else if (gamepad1.b) {
                pto.position = .95
            }
        }
    }
}
