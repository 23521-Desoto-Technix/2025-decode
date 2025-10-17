package org.firstinspires.ftc.teamcode.manual

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class servoTest : LinearOpMode() {
    override fun runOpMode() {
        val servo0 = hardwareMap.servo.get("weThePeople")
        val servo1 = hardwareMap.servo.get("ofTheseUnitedStates")
        waitForStart()
        while (opModeIsActive()) {
            servo0.position = (gamepad1.left_stick_y + 1.0) / 2
            servo1.position = (gamepad1.right_stick_y + 1.0) / 2
            telemetry.addData(servo0.deviceName + ", port: " + servo0.portNumber, servo0.position)
            telemetry.addData(servo1.deviceName + ", port: " + servo1.portNumber, servo1.position)
            telemetry.update()
        }
    }
}