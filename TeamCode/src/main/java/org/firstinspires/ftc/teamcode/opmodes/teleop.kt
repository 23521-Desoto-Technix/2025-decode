package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.bindings.range
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Tube

@TeleOp
class teleop : NextFTCOpMode() {
  init {
    addComponents(
        BulkReadComponent,
        BindingsComponent,
        PedroComponent(Constants::createFollower),
        SubsystemComponent(Tube),
    )
  }

  var rotatedForward = 0.0
  var rotatedStrafe = 0.0
  var rotatedTurn = 0.0

  override fun onInit() {
    // Initialize opmode
  }

  override fun onWaitForStart() {
    // Wait for start
  }

  override fun onStartButtonPressed() {
    val driverControlled =
        PedroDriverControlled(
            range { rotatedForward },
            range { rotatedStrafe },
            range { rotatedTurn },
        )
    driverControlled()
    val intake = button { gamepad1.circle }.whenBecomesTrue { Tube.intakeAll.schedule() }
    val stopIntake = button { gamepad1.cross }.whenBecomesTrue { Tube.stopAll.schedule() }
    val shoot = button { gamepad1.square }.whenBecomesTrue { Tube.shootAll.schedule() }
  }

  override fun onUpdate() {
    telemetry.addData("X", PedroComponent.follower.pose.x)
    telemetry.addData("Y", PedroComponent.follower.pose.y)
    telemetry.addData("Heading", PedroComponent.follower.pose.heading)
    BindingManager.update()
    telemetry.update()
    rotatedForward = -gamepad1.left_stick_y.toDouble()
    rotatedStrafe = -gamepad1.left_stick_x.toDouble()
    rotatedTurn = -gamepad1.right_stick_x.toDouble()
  }

  override fun onStop() {
    BindingManager.reset()
  }
}
