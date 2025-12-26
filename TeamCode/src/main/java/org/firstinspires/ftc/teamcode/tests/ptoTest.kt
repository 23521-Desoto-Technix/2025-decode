package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "PTO Test")
class ptoTest : LinearOpMode() {
  override fun runOpMode() {
    val backRight = hardwareMap.dcMotor["backRight"]
    val frontLeft = hardwareMap.dcMotor["frontLeft"]
    val backLeft = hardwareMap.dcMotor["backLeft"]
    val frontRight = hardwareMap.dcMotor["frontRight"]

    val pto = hardwareMap.servo["pto"]
    val left = hardwareMap.dcMotor["left"]
    val right = hardwareMap.dcMotor["right"]
    right.direction = DcMotorSimple.Direction.REVERSE
    var power = 0.0
    waitForStart()
    while (opModeIsActive()) {

      val stickValue = gamepad2.left_stick_y.toDouble()
      backRight.power = stickValue
      frontLeft.power = stickValue
      backLeft.power = -stickValue
      frontRight.power = -stickValue

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
