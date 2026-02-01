package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.Turret

@TeleOp(name = "Turret Test")
class turretTest : NextFTCOpMode() {
  init {
    addComponents(
        SubsystemComponent(Turret),
        BulkReadComponent,
        BindingsComponent,
    )
  }

  var angle = 0.0
  var usingRaw = false
  var raw = 0.5

  override fun onUpdate() {
    if (gamepad1.aWasPressed()) {
      usingRaw = !usingRaw
    }

    if (usingRaw) {
      if (gamepad1.dpad_up) {
        raw += 0.01
      }
      if (gamepad1.dpad_down) {
        raw -= 0.01
      }
      if (gamepad1.dpad_left) {
        raw += 0.001
      }
      if (gamepad1.dpad_right) {
        raw -= 0.001
      }
      raw = raw.coerceIn(0.0, 1.0)
      telemetry.addData("Mode", "Raw")
      telemetry.addData("Raw Position", raw)
      Turret.setRawPosition(raw)
    } else {
      if (gamepad1.dpad_up) {
        angle += 15.0
      }
      if (gamepad1.dpad_down) {
        angle -= 15.0
      }
      if (gamepad1.dpad_left) {
        angle += 1.0
      }
      if (gamepad1.dpad_right) {
        angle -= 1.0
      }
      telemetry.addData("Mode", "Angle")
      telemetry.addData("Target Angle", angle)
      Turret.setTargetAngle(angle.deg)
    }
    telemetry.update()
  }
}
