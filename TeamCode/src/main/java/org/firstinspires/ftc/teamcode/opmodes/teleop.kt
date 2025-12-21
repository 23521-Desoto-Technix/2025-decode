package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.BindingManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp
class teleop : NextFTCOpMode() {
  init {
    addComponents(
        BulkReadComponent,
        BindingsComponent,
        PedroComponent(Constants::createFollower),
    )
  }

  private val frontLeft = MotorEx("frontLeft").brakeMode().reversed()
  private val frontRight = MotorEx("frontRight").brakeMode()
  private val backLeft = MotorEx("backLeft").brakeMode().reversed()
  private val backRight = MotorEx("backRight").brakeMode()

  override fun onInit() {
    // Initialize opmode
  }

  override fun onWaitForStart() {
    // Wait for start
  }

  override fun onStartButtonPressed() {
    val driverControlled =
        MecanumDriverControlled(
            frontLeft,
            frontRight,
            backLeft,
            backRight,
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX,
        )
      driverControlled()

  }

  override fun onUpdate() {
    telemetry.addData("X", PedroComponent.follower.pose.x)
    telemetry.addData("Y", PedroComponent.follower.pose.y)
    telemetry.addData("Heading", PedroComponent.follower.pose.heading)
    BindingManager.update()
    telemetry.update()
  }

  override fun onStop() {
    BindingManager.reset()
  }
}
