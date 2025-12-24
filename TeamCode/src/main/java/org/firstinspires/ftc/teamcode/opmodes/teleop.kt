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
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Tube

@TeleOp
class teleop : NextFTCOpMode() {
  init {
    addComponents(
        BulkReadComponent,
        BindingsComponent,
        PedroComponent(Constants::createFollower),
        SubsystemComponent(Tube, Shooter, Flywheel, Hood),
    )
  }

  var rotatedForward = 0.0
  var rotatedStrafe = 0.0
  var rotatedTurn = 0.0

  private lateinit var backRight: com.qualcomm.robotcore.hardware.DcMotor
  private lateinit var frontLeft: com.qualcomm.robotcore.hardware.DcMotor
  private lateinit var backLeft: com.qualcomm.robotcore.hardware.DcMotor
  private lateinit var frontRight: com.qualcomm.robotcore.hardware.DcMotor

  override fun onInit() {
    backRight = hardwareMap.dcMotor["backRight"]
    frontLeft = hardwareMap.dcMotor["frontLeft"]
    backLeft = hardwareMap.dcMotor["backLeft"]
    frontRight = hardwareMap.dcMotor["frontRight"]
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
    val shootAll = button { gamepad1.triangle }.whenBecomesTrue { Tube.shootAll().schedule() }
    val shootAllSlow = button { gamepad1.square }.whenBecomesTrue { Tube.shootAll(0.7).schedule() }
    val flywheelLong =
        button { gamepad1.dpad_up }
            .whenBecomesTrue { Flywheel.enable().then(Flywheel.setSpeed(2_200.0)).schedule() }
    val flywheelShort =
        button { gamepad1.dpad_down }
            .whenBecomesTrue { Flywheel.enable().then(Flywheel.setSpeed(2_100.0)).schedule() }
    val flywheelTesting =
        button { gamepad1.dpad_left }
            .whenBecomesTrue { Flywheel.enable().then(Flywheel.setSpeed(500.0)).schedule() }
    val flywheelOff =
        button { gamepad1.dpad_right }.whenBecomesTrue { Flywheel.disable().schedule() }
    val hoodUp = button { gamepad1.left_bumper }.whenBecomesTrue { Hood.bumpUp().schedule() }
    val hoodDown = button { gamepad1.right_bumper }.whenBecomesTrue { Hood.bumpDown().schedule() }
  }

  override fun onUpdate() {
    telemetry.addData("X", PedroComponent.follower.pose.x)
    telemetry.addData("Y", PedroComponent.follower.pose.y)
    telemetry.addData("Heading", PedroComponent.follower.pose.heading)
    BindingManager.update()
    telemetry.update()

    if (gamepad2.left_stick_y != 0f) {
      rotatedForward = 0.0
      rotatedStrafe = 0.0
      rotatedTurn = 0.0

      val stickValue = gamepad2.left_stick_y.toDouble()
      backRight.power = stickValue
      frontLeft.power = stickValue
      backLeft.power = -stickValue
      frontRight.power = -stickValue
    } else {
      rotatedForward = -gamepad1.left_stick_y.toDouble()
      rotatedStrafe = -gamepad1.left_stick_x.toDouble()
      rotatedTurn = -gamepad1.right_stick_x.toDouble()
    }
  }

  override fun onStop() {
    BindingManager.reset()
  }
}
