package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "PTO Test")
class ptoTest : LinearOpMode() {
  override fun runOpMode() {
    val backRight = hardwareMap.dcMotor["backRight"]
    val frontLeft = hardwareMap.dcMotor["frontLeft"]
    val backLeft = hardwareMap.dcMotor["backLeft"]
    val frontRight = hardwareMap.dcMotor["frontRight"]
    val pto = hardwareMap.servo["pto"]
    waitForStart()
    while (opModeIsActive()) {

      val stickValue = gamepad2.left_stick_y.toDouble()
      backRight.power = stickValue
      frontLeft.power = stickValue
      backLeft.power = -stickValue
      frontRight.power = -stickValue

      if (gamepad1.a) {
        pto.position = 0.0
      } else if (gamepad1.b) {
        pto.position = .95
      }
    }
  }
}
