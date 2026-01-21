package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Turret Test")
@Disabled
class turretTest : LinearOpMode() {
  override fun runOpMode() {
    val turretLeft = hardwareMap.servo["turretLeft"]
    val turretRight = hardwareMap.servo["turretRight"]
    val encoder = hardwareMap.analogInput["turretEncoder"]

    var offset = 0.0

    waitForStart()
    while (opModeIsActive()) {
      if (gamepad1.dpadLeftWasPressed()) {
        offset -= 0.01
      }
      if (gamepad1.dpadRightWasPressed()) {
        offset += 0.01
      }
      if (gamepad1.circleWasPressed()) {
        offset = 0.0
      }
      if (gamepad1.leftBumperWasPressed()) {
        offset -= 0.001
      }
      if (gamepad1.rightBumperWasPressed()) {
        offset += 0.001
      }

      turretLeft.position = 0.5
      turretRight.position = 0.5 + offset

      telemetry.addData("Encoder Value", (encoder.voltage / 3.3 * 2) - 1)
      telemetry.addData("Offset", offset)
      telemetry.update()
    }
  }
}
