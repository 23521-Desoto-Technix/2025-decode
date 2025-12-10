package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "PTO Test")
class ptoTest : LinearOpMode() {
    override fun runOpMode() {
        val pto = hardwareMap.servo["pto"]
        val left = hardwareMap.dcMotor["left"]
        val right = hardwareMap.dcMotor["right"]
        waitForStart()
        while (opModeIsActive()) {
            left.power = -gamepad1.left_stick_y.toDouble()
            right.power = -gamepad1.right_stick_y.toDouble()
            if (gamepad1.a) {
                pto.position = 0.0
            } else if (gamepad1.b) {
                pto.position = 1.0
            }
        }
    }
}
