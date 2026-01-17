package org.firstinspires.ftc.teamcode.opmodes

import android.util.Size
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.bindings.range
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.BotConstants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Tube
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.BotState
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
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
        SubsystemComponent(Tube, Shooter, Flywheel, Hood, Turret),
    )
  }

  var rotatedForward = 0.0
  var rotatedStrafe = 0.0
  var rotatedTurn = 0.0

  var ignorePinpoint = false

  var flywheelTargetSpeed = 0.0

  var headingLocked = false

  var slowMode = false

  val headingPID = controlSystem { posPid(0.0085, 0.0, 0.0) }

  private lateinit var backRight: com.qualcomm.robotcore.hardware.DcMotor
  private lateinit var frontLeft: com.qualcomm.robotcore.hardware.DcMotor
  private lateinit var backLeft: com.qualcomm.robotcore.hardware.DcMotor
  private lateinit var frontRight: com.qualcomm.robotcore.hardware.DcMotor
  private lateinit var pto: Servo

  lateinit var aprilTag: AprilTagProcessor
  lateinit var portal: VisionPortal

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
    pto = hardwareMap.servo["pto"]
    telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
    telemetry.msTransmissionInterval = 25

    aprilTag =
        AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .setLensIntrinsics(
                BotConstants.CAMERA_LENS_FX,
                BotConstants.CAMERA_LENS_FY,
                BotConstants.CAMERA_LENS_CX,
                BotConstants.CAMERA_LENS_CY,
            )
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
            // ... these parameters are fx, fy, cx, cy.
            .build()

    portal =
        VisionPortal.Builder()
            .setCamera(hardwareMap.get<WebcamName?>(WebcamName::class.java, "turretCamera"))
            .setCameraResolution(
                Size(BotConstants.CAMERA_RESOLUTION_WIDTH, BotConstants.CAMERA_RESOLUTION_HEIGHT)
            )
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(aprilTag)
            .build()
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
    BotState.enabled = true
    val startingPose =
        when (BotState.alliance) {
          Alliance.RED -> Pose(72.0, 72.0, 90.0.deg.inRad)
          Alliance.BLUE -> Pose(72.0, 72.0, 90.0.deg.inRad)
          Alliance.UNKNOWN -> Pose(72.0, 7.0, 90.0.deg.inRad)
        }
    if (BotState.pose != null) {
      PedroComponent.follower.pose = BotState.pose!!
    } else {
      PedroComponent.follower.pose = startingPose
    }

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
        button { gamepad1.dpad_up || gamepad2.dpad_up }
            .whenBecomesTrue {
              flywheelTargetSpeed = 2_050.0
              Hood.position = 0.65
              Flywheel.enable().then(Flywheel.setSpeed(2_050.0)).schedule()
            }
    val flywheelShort =
        button { gamepad1.dpad_down || gamepad2.dpad_down }
            .whenBecomesTrue {
              flywheelTargetSpeed = 1_600.0
              Hood.position = 0.45
              Flywheel.enable().then(Flywheel.setSpeed(1_600.0)).schedule()
            }
    val flywheelTesting =
        button { gamepad1.dpad_left || gamepad1.dpad_left }
            .whenBecomesTrue {
              flywheelTargetSpeed = 500.0
              Flywheel.enable().then(Flywheel.setSpeed(500.0)).schedule()
            }
    val flywheelOff =
        button { gamepad1.dpad_right || gamepad2.dpad_right }
            .whenBecomesTrue { Flywheel.disable().schedule() }
    val flywheelSpeedUp =
        button { gamepad2.left_trigger > 0.5 }
            .whenBecomesTrue {
              flywheelTargetSpeed += 100.0
              Flywheel.enable().then(Flywheel.setSpeed(flywheelTargetSpeed)).schedule()
            }
    val flywheelSpeedDown =
        button { gamepad2.right_trigger > 0.5 }
            .whenBecomesTrue {
              flywheelTargetSpeed = maxOf(0.0, flywheelTargetSpeed - 100.0)
              Flywheel.enable().then(Flywheel.setSpeed(flywheelTargetSpeed)).schedule()
            }
    val hoodUp = button { gamepad2.left_bumper }.whenBecomesTrue { Hood.bumpUp().schedule() }
    val hoodDown = button { gamepad2.right_bumper }.whenBecomesTrue { Hood.bumpDown().schedule() }
    val driveCancel =
        button { abs(gamepad2.left_stick_y) > 0.1 }
            .whenBecomesTrue { driverControlled.cancel() }
            .whenBecomesFalse { driverControlled.schedule() }
    val ptoOn =
        button { gamepad2.circle && gamepad2.ps }
            .whenBecomesTrue {
              if (BotState.enabled) {
                pto.position = 0.95
              }
            }
    val ptoOff =
        button { gamepad2.cross }
            .whenBecomesTrue {
              if (BotState.enabled) {
                pto.position = 0.0
              }
            }
    val ignorePinpointToggle =
        button { gamepad2.square }.whenBecomesTrue { ignorePinpoint = !ignorePinpoint }
    val headingLock =
        button { gamepad1.right_bumper }
            .whenTrue { headingLocked = true }
            .whenFalse { headingLocked = false }
    val slow =
        button { gamepad1.left_bumper }.whenTrue { slowMode = true }.whenFalse { slowMode = false }
  }

  override fun onUpdate() {
    var targetPose = Pose(144.0, 144.0, 0.0)
    if (BotState.alliance == Alliance.BLUE) {
      targetPose = Pose(0.0, 144.0, 0.0)
    }
    val currentX = PedroComponent.follower.pose.x
    val currentY = PedroComponent.follower.pose.y
    val deltaX = targetPose.x - currentX
    val deltaY = targetPose.y - currentY
    val absoluteAngleToTarget = atan2(deltaY, deltaX).rad
    val relativeAngleToTarget =
        (PedroComponent.follower.pose.heading.rad - absoluteAngleToTarget + 180.deg).normalized

    if (ignorePinpoint) {
      if (((System.currentTimeMillis() / 500) % 2).toInt() == 0) {
        telemetry.addLine(
            "<span style=\"background-color: yellow; color: black;\">&nbsp;&nbsp;!!&nbsp;&nbsp;IGNORING PINPOINT&nbsp;&nbsp;!!&nbsp;&nbsp;</span>"
        )
      } else {
        telemetry.addLine("&nbsp;&nbsp;!!&nbsp;&nbsp;IGNORING PINPOINT&nbsp;&nbsp;!!&nbsp;&nbsp;")
      }
    }

    telemetry.addData("X", PedroComponent.follower.pose.x)
    telemetry.addData("Y", PedroComponent.follower.pose.y)
    telemetry.addData("Heading", PedroComponent.follower.pose.heading)
    telemetry.addData("Angle to (144, 144)", relativeAngleToTarget.inDeg)
    telemetry.addData("Flywheel Target Speed", flywheelTargetSpeed)
    telemetry.addData("Hood position", Hood.position)
    /* HTML telemetry reference
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
        "<span style=\"background-color: #FF0000; color: white;\">Red background</span>"
    )
    telemetry.addLine(
        "<span style=\"background-color: #0000FF; color: white;\">Blue background</span>"
    )*/

    var tatag = 0

    if (BotState.alliance == Alliance.RED) {
      tatag = BotConstants.RED_ALLIANCE_APRILTAG_ID
    } else if (BotState.alliance == Alliance.BLUE) {
      tatag = BotConstants.BLUE_ALLIANCE_APRILTAG_ID
    }

    var targetTagBearing: Double? = null
    for (detection in aprilTag.detections) {
      if (detection.id == tatag) {
        targetTagBearing = detection.ftcPose.bearing
        break
      }
    }

    if (targetTagBearing != null) {
      telemetry.addData("Target AprilTag Yaw", targetTagBearing)
    }

    if (abs(gamepad2.left_stick_y) > 0.1) {
      if (BotState.enabled) {
        backRight.power = gamepad2.left_stick_y.toDouble()
        frontRight.power = -gamepad2.left_stick_y.toDouble() * 0.75
        backLeft.power = gamepad2.left_stick_y.toDouble()
        frontLeft.power = -gamepad2.left_stick_y.toDouble() * 0.75
      } else {
        backRight.power = 0.0
        frontRight.power = 0.0
        backLeft.power = 0.0
        frontLeft.power = 0.0
      }
    } else if (!BotState.enabled) {
      backRight.power = 0.0
      frontRight.power = 0.0
      backLeft.power = 0.0
      frontLeft.power = 0.0
    }
    BindingManager.update()
    telemetry.update()
    var rotateBy = -PedroComponent.follower.pose.heading.rad
    if (BotState.alliance == Alliance.BLUE) {
      rotateBy = (rotateBy + 180.deg).normalized
    }
    if (ignorePinpoint) {
      rotateBy = 0.0.deg
    }
    telemetry.addData("Current angle", rotateBy.normalized.inDeg)
    val rotated =
        rotateJoystickInput(
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            rotateBy,
        )
    rotatedForward = rotated.first
    rotatedStrafe = rotated.second

    if (slowMode) {
      rotatedForward *= 0.5
      rotatedStrafe *= 0.5
    }

    if (headingLocked) {
      if (BotState.alliance == Alliance.RED) {
        headingPID.goal = KineticState(135.0, 0.0)
      } else if (BotState.alliance == Alliance.BLUE) {
        headingPID.goal = KineticState(45.0, 0.0)
      }

      rotatedTurn =
          headingPID.calculate(
              KineticState(
                  PedroComponent.follower.pose.heading.rad.inDeg,
                  PedroComponent.follower.angularVelocity.rad.inDeg,
              )
          )
    } else {
      rotatedTurn = -gamepad1.right_stick_x.toDouble()
    }
    telemetry.addData("turn thingy", rotatedTurn)
    if (!BotState.enabled) {
      rotatedForward = 0.0
      rotatedStrafe = 0.0
      rotatedTurn = 0.0
      return
    }

    /*
    if (!ignorePinpoint) {
      Turret.setTargetAngle(-relativeAngleToTarget)
    } else if (targetTagBearing == null) {
      Turret.setTargetAngle(0.0.deg)
    } else {
      Turret.setTargetAngle(Turret.currentAngle - targetTagBearing.deg)
    }*/
    if (!ignorePinpoint) {
      Turret.setTargetAngle(-relativeAngleToTarget)
    } else {
      Turret.setTargetAngle(0.0.deg)
    }
  }

  override fun onStop() {
    BindingManager.reset()
  }
}
