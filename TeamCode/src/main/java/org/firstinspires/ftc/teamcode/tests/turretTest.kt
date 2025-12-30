package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Turret Test")
class turretTest : LinearOpMode() {
  override fun runOpMode() {
    val turretLeft = hardwareMap.servo["turretLeft"]
    val turretRight = hardwareMap.servo["turretRight"]
    val encoder = hardwareMap.analogInput["turretEncoder"]

    waitForStart()
    while (opModeIsActive()) {
      val joystickValue = gamepad1.left_stick_y.toDouble()
      val servoPower = (joystickValue + 1.0) / 2.0

      turretLeft.position = servoPower
      turretRight.position = servoPower

      telemetry.addData("Servo Power", servoPower)
      telemetry.addData("Encoder Value", (encoder.voltage / 3.3 * 2) - 1)
      telemetry.update()
    }
  }
}
