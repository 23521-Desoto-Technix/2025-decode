package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.bindings.range
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Tube
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.BotState
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

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

  var flywheelTargetSpeed = 0.0

  private lateinit var backRight: com.qualcomm.robotcore.hardware.DcMotor
  private lateinit var frontLeft: com.qualcomm.robotcore.hardware.DcMotor
  private lateinit var backLeft: com.qualcomm.robotcore.hardware.DcMotor
  private lateinit var frontRight: com.qualcomm.robotcore.hardware.DcMotor

  fun rotateJoystickInput(
      forward: Double,
      strafe: Double,
      angle: dev.nextftc.core.units.Angle,
  ): Pair<Double, Double> {
    val angleRadians = angle.value
    val rotatedForward = forward * cos(angleRadians) - strafe * sin(angleRadians)
    val rotatedStrafe = forward * sin(angleRadians) + strafe * cos(angleRadians)
    return Pair(rotatedForward, rotatedStrafe)
  }

  override fun onInit() {
    backRight = hardwareMap.dcMotor["backRight"]
    frontLeft = hardwareMap.dcMotor["frontLeft"]
    backLeft = hardwareMap.dcMotor["backLeft"]
    frontRight = hardwareMap.dcMotor["frontRight"]
    telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
    telemetry.msTransmissionInterval = 25
    PedroComponent.follower.pose = Pose(72.0, 72.0, 90.0.deg.inRad)
  }

  override fun onWaitForStart() {
    BindingManager.layer = "init"

    val selectRed =
        button { gamepad1.circle }
            .inLayer("init")
            .whenBecomesTrue { BotState.alliance = Alliance.RED }
    val selectBlue =
        button { gamepad1.cross }
            .inLayer("init")
            .whenBecomesTrue { BotState.alliance = Alliance.BLUE }
    val allianceDisplay =
        when (BotState.alliance) {
          Alliance.RED ->
              "<span style=\"background-color: #FF0000; color: white;\">&nbsp;&nbsp;RED&nbsp;&nbsp;</span>"
          Alliance.BLUE ->
              "<span style=\"background-color: #0000FF; color: white;\">&nbsp;&nbsp;BLUE&nbsp;&nbsp;</span>"
          Alliance.UNKNOWN -> {

            if (((System.currentTimeMillis() / 500) % 2).toInt() == 0) {
              "<span style=\"background-color: yellow; color: black;\">&nbsp;&nbsp;!!&nbsp;&nbsp;UNKNOWN&nbsp;&nbsp;!!&nbsp;&nbsp;</span>"
            } else {
              "&nbsp;&nbsp;!!&nbsp;&nbsp;UNKNOWN&nbsp;&nbsp;!!&nbsp;&nbsp;"
            }
          }
        }

    telemetry.addLine(allianceDisplay)
    telemetry.addLine("RED: Circle ●")
    telemetry.addLine("BLUE: Cross ✕")

    BindingManager.update()
    telemetry.update()
  }

  override fun onStartButtonPressed() {
    val driverControlled =
        PedroDriverControlled(
            range { rotatedForward },
            range { rotatedStrafe },
            range { rotatedTurn },
        )
    driverControlled()
    BindingManager.layer = null
    val intake = button { gamepad1.circle }.whenBecomesTrue { Tube.intakeAll.schedule() }
    val stopIntake = button { gamepad1.cross }.whenBecomesTrue { Tube.stopAll.schedule() }
    val shootAll = button { gamepad1.triangle }.whenBecomesTrue { Tube.shootAll().schedule() }
    val shootAllSlow = button { gamepad1.square }.whenBecomesTrue { Tube.shootAll(0.8).schedule() }
    val flywheelLong =
        button { gamepad1.dpad_up }
            .whenBecomesTrue {
              flywheelTargetSpeed = 2_450.0
              Flywheel.enable().then(Flywheel.setSpeed(2_200.0)).schedule()
            }
    val flywheelShort =
        button { gamepad1.dpad_down }
            .whenBecomesTrue {
              flywheelTargetSpeed = 2_100.0
              Flywheel.enable().then(Flywheel.setSpeed(2_100.0)).schedule()
            }
    val flywheelTesting =
        button { gamepad1.dpad_left }
            .whenBecomesTrue {
              flywheelTargetSpeed = 500.0
              Flywheel.enable().then(Flywheel.setSpeed(500.0)).schedule()
            }
    val flywheelOff =
        button { gamepad1.dpad_right }.whenBecomesTrue { Flywheel.disable().schedule() }
    val flywheelSpeedUp =
        button { gamepad1.left_trigger > 0.5 }
            .whenBecomesTrue {
              flywheelTargetSpeed += 100.0
              Flywheel.enable().then(Flywheel.setSpeed(flywheelTargetSpeed)).schedule()
            }
    val flywheelSpeedDown =
        button { gamepad1.right_trigger > 0.5 }
            .whenBecomesTrue {
              flywheelTargetSpeed = maxOf(0.0, flywheelTargetSpeed - 100.0)
              Flywheel.enable().then(Flywheel.setSpeed(flywheelTargetSpeed)).schedule()
            }
    val hoodUp = button { gamepad1.left_bumper }.whenBecomesTrue { Hood.bumpUp().schedule() }
    val hoodDown = button { gamepad1.right_bumper }.whenBecomesTrue { Hood.bumpDown().schedule() }
  }

  override fun onUpdate() {
    val targetPose = Pose(144.0, 144.0, 0.0)
    val currentX = PedroComponent.follower.pose.x
    val currentY = PedroComponent.follower.pose.y
    val deltaX = targetPose.x - currentX
    val deltaY = targetPose.y - currentY
    val angleToTarget = Math.toDegrees(atan2(deltaY, deltaX))

    telemetry.addData("X", PedroComponent.follower.pose.x)
    telemetry.addData("Y", PedroComponent.follower.pose.y)
    telemetry.addData("Heading", PedroComponent.follower.pose.heading)
    telemetry.addData("Angle to (144, 144)", angleToTarget)
    telemetry.addData("Flywheel Target Speed", flywheelTargetSpeed)
    telemetry.addData("Hood position", Hood.position)

    telemetry.addLine("<b>Bold text</b>")
    telemetry.addLine("<i>Italic text</i>")
    telemetry.addLine("<u>Underlined text</u>")
    telemetry.addLine("<font color=\"#FF0000\">Red text</font>")
    telemetry.addLine("<font color=\"#00FF00\">Green text</font>")
    telemetry.addLine("<b><i>Bold and italic</i></b>")
    telemetry.addLine(
        "<span style=\"background-color: yellow; color: black;\">Yellow background</span>"
    )
    telemetry.addLine(
        "<span style=\"background-color: #FF0000; color: white;\">Red-ish background</span>"
    )
    telemetry.addLine(
        "<span style=\"background-color: #0000FF; color: white;\">Blue background</span>"
    )

    BindingManager.update()
    telemetry.update()
    val rotateBy = -PedroComponent.follower.pose.heading.rad
    telemetry.addData("Current angle", rotateBy.normalized.inDeg)
    val rotated =
        rotateJoystickInput(
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            rotateBy,
        )
    rotatedForward = rotated.first
    rotatedStrafe = rotated.second
    rotatedTurn = -gamepad1.right_stick_x.toDouble()
  }

  override fun onStop() {
    BindingManager.reset()
  }
}
