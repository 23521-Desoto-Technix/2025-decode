package org.firstinspires.ftc.teamcode.opmodes

import android.util.Size
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.bindings.range
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.BotConstants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Indexer
import org.firstinspires.ftc.teamcode.subsystems.Lights
import org.firstinspires.ftc.teamcode.subsystems.LightsState
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.atan2
import kotlin.time.Duration.Companion.seconds

@TeleOp
class teleop : NextFTCOpMode() {
  init {
    addComponents(
        SubsystemComponent(Shooter, Indexer, Lights, Turret, Hood),
        BulkReadComponent,
        BindingsComponent,
        PedroComponent(Constants::createFollower),
    )
  }

  companion object {
    // Color detection HSV thresholds
    const val PURPLE_HUE_MIN = 200f
    const val PURPLE_HUE_MAX = 245f
    const val GREEN_HUE_MIN = 150f
    const val GREEN_HUE_MAX = 170f
    const val SATURATION_THRESHOLD = 140f

    // Turret lock buffer
    const val LOCK_BUFFER_MS = 150

    // Gamepad thresholds
    const val TRIGGER_THRESHOLD = 0.5
    const val SPEED_MULTIPLIER_SLOW = 0.5
    const val SPEED_MULTIPLIER_NORMAL = 1.0

    // Hood adjustment
    const val HOOD_ADJUSTMENT_STEP = 0.1

    // Rumble duration
    const val RUMBLE_DURATION_MS = 250

    // Rotation compensation multiplier
    const val ROTATION_COMP_MULTIPLIER = 150.0

    // HSV conversion constants
    const val HSV_DELTA_THRESHOLD = 1e-6f
    const val HSV_HUE_SEGMENTS = 6f
    const val HSV_HUE_GREEN_OFFSET = 2f
    const val HSV_HUE_BLUE_OFFSET = 4f
    const val HSV_HUE_MULTIPLIER = 60f
    const val HSV_HUE_MAX = 360f
    const val HSV_SCALE_MAX = 255f
    const val HSV_CLAMP_MIN = 0f
    const val HSV_CLAMP_MAX = 1f

    // Feeder delay for sequential group (seconds)
    const val FEEDER_DELAY_SECONDS = 0.2
  }

  private lateinit var intakeBreakBeam: DigitalChannel
  private lateinit var leftBreakBeam: DigitalChannel
  private lateinit var rightBreakBeam: DigitalChannel
  private lateinit var intakeColor: RevColorSensorV3

  lateinit var aprilTag: AprilTagProcessor
  lateinit var portal: VisionPortal

  var targetPose = Pose(72.0, 144.0, 0.0)
  var targetAprilTag = 0

  var speedMultiplier = 1.0

  var alliance = Alliance.UNKNOWN

  var lastLockTime: Long = 0

  override fun onInit() {
    intakeBreakBeam = hardwareMap.get(DigitalChannel::class.java, "intakeBreakBeam")
    intakeBreakBeam.mode = DigitalChannel.Mode.INPUT
    leftBreakBeam = hardwareMap.get(DigitalChannel::class.java, "leftBreakBeam")
    leftBreakBeam.mode = DigitalChannel.Mode.INPUT
    rightBreakBeam = hardwareMap.get(DigitalChannel::class.java, "rightBreakBeam")
    rightBreakBeam.mode = DigitalChannel.Mode.INPUT
    intakeColor = hardwareMap.get(RevColorSensorV3::class.java, "intakeColor")
    Indexer.intakeBreakBeam = intakeBreakBeam
    Indexer.leftBreakBeam = leftBreakBeam
    Indexer.rightBreakBeam = rightBreakBeam
    Indexer.indexerToSlot(0).schedule()
    if (blackboard["pose"] != null && blackboard["pose"] is Pose) {
      PedroComponent.follower.pose = blackboard["pose"] as Pose?
    } else {
      PedroComponent.follower.pose = BotConstants.FIELD_CENTER
    }
    if (blackboard["alliance"] != null && blackboard["alliance"] is Alliance) {
      alliance = blackboard["alliance"] as Alliance
    }
    PedroComponent.follower.breakFollowing()

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
    if (gamepad1.circle) {
      alliance = Alliance.RED
    } else if (gamepad1.cross) {
      alliance = Alliance.BLUE
    }
    Lights.state = alliance.lightsState()
  }

  override fun onStartButtonPressed() {
    val shooterFar =
        button { gamepad2.right_bumper }
            .whenBecomesTrue { Shooter.setSpeed(BotConstants.SHOOTER_SPEED_FAR).schedule() }
    val shooterClose =
        button { gamepad2.left_bumper }
            .whenBecomesTrue { Shooter.setSpeed(BotConstants.SHOOTER_SPEED_CLOSE).schedule() }
    val shooterOff =
        button { gamepad2.left_trigger > TRIGGER_THRESHOLD }
            .whenBecomesTrue { Shooter.setSpeed(BotConstants.SHOOTER_SPEED_OFF).schedule() }
    val intakeForward =
        button { gamepad2.circle }
            .whenBecomesTrue {
              Indexer.setIntakePower(
                      if (Indexer.intakePower == BotConstants.INTAKE_POWER_FORWARD)
                          BotConstants.INTAKE_POWER_OFF
                      else BotConstants.INTAKE_POWER_FORWARD
                  )
                  .schedule()
            }
    val intakeReverse =
        button { gamepad2.cross }
            .whenBecomesTrue {
              Indexer.setIntakePower(
                      if (Indexer.intakePower == BotConstants.INTAKE_POWER_REVERSE)
                          BotConstants.INTAKE_POWER_OFF
                      else BotConstants.INTAKE_POWER_REVERSE
                  )
                  .schedule()
            }
    val spindexerBumpNext =
        button { gamepad2.dpad_left } whenBecomesTrue { Indexer.toNextSlot().schedule() }
    val spindexerBumpPrevious =
        button { gamepad2.dpad_right } whenBecomesTrue { Indexer.toPreviousSlot().schedule() }
    val spindexerReset =
        button { gamepad2.dpad_down } whenBecomesTrue { Indexer.indexerToSlot(0).schedule() }

    val feed =
        button { gamepad2.triangle } whenTrue
            {
              Indexer.feed().schedule()
            } whenBecomesFalse
            {
              SequentialGroup(
                      Indexer.unFeed(),
                      Delay(FEEDER_DELAY_SECONDS.seconds),
                      Indexer.toNextSlot(),
                  )
                  .schedule()
            }
    val latch =
        button { gamepad2.square }
            .toggleOnBecomesTrue()
            .whenBecomesTrue { Indexer.latchDown().schedule() }
            .whenBecomesFalse { Indexer.latchUp().schedule() }
    val hoodUp =
        button { gamepad1.dpad_up }
            .whenBecomesTrue {
              Hood.setPosition(
                      (Hood.position + HOOD_ADJUSTMENT_STEP).coerceAtMost(SPEED_MULTIPLIER_NORMAL)
                  )
                  .schedule()
            }
    val hoodDown =
        button { gamepad1.dpad_down }
            .whenBecomesTrue {
              Hood.setPosition((Hood.position - HOOD_ADJUSTMENT_STEP).coerceAtLeast(0.0)).schedule()
            }
    /*
    val turretRight =
        button { gamepad1.dpad_right }
            .whenBecomesTrue { Turret.setAngle((Turret.targetAngle + 45.0).deg, true).schedule() }
    val turretLeft =
        button { gamepad1.dpad_left }
            .whenBecomesTrue { Turret.setAngle((Turret.targetAngle - 45.0).deg, true).schedule() }
    val turretCenter =
        button { gamepad1.circle }.whenBecomesTrue { Turret.setAngle(0.0.deg, true).schedule() }
        */
    val rumble =
        button { !leftBreakBeam.state || !rightBreakBeam.state }
            .whenBecomesTrue {
              gamepad1.rumble(RUMBLE_DURATION_MS)
              gamepad2.rumble(RUMBLE_DURATION_MS)
            }

    val gateDisplay =
        button { Indexer.latched }
            .whenTrue { gamepad2.setLedColor(0.0, 1.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS) }
            .whenFalse { gamepad2.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS) }
    val speedToggle =
        Gamepads.gamepad1.rightTrigger
            .atLeast(TRIGGER_THRESHOLD)
            .whenBecomesTrue {
              InstantCommand { speedMultiplier = SPEED_MULTIPLIER_SLOW }.schedule()
            }
            .whenBecomesFalse {
              InstantCommand { speedMultiplier = SPEED_MULTIPLIER_NORMAL }.schedule()
            }
    val driverControlled =
        PedroDriverControlled(
            range { -gamepad1.left_stick_y.toDouble() * speedMultiplier },
            range { -gamepad1.left_stick_x.toDouble() * speedMultiplier },
            range { -gamepad1.right_stick_x.toDouble() * speedMultiplier },
            false,
        )
    driverControlled()
  }

  override fun onUpdate() {
    val normalized = intakeColor.normalizedColors
    val r = normalized.red
    val g = normalized.green
    val b = normalized.blue // 215 +- 15 for purp, 160 +- 10 for green

    val hsv = rgbToHsv(r, g, b)
    if (hsv[0] > PURPLE_HUE_MIN && hsv[0] < PURPLE_HUE_MAX && hsv[1] > SATURATION_THRESHOLD) {
      Lights.state = LightsState.ARTIFACT_PURPLE
    } else if (hsv[0] > GREEN_HUE_MIN && hsv[0] < GREEN_HUE_MAX && hsv[1] > SATURATION_THRESHOLD) {
      Lights.state = LightsState.ARTIFACT_GREEN
    } else {
      Lights.state = LightsState.OFF
    }
    telemetry.addData("Status", intakeColor.status())
    telemetry.addData("Color H", String.format("%.1f", hsv[0]))
    telemetry.addData("Color S", String.format("%.0f", hsv[1]))
    telemetry.addData("Color V", String.format("%.0f", hsv[2]))
    telemetry.addData("X", PedroComponent.follower.pose.x)
    telemetry.addData("Y", PedroComponent.follower.pose.y)
    telemetry.addData("Heading", PedroComponent.follower.pose.heading)
    telemetry.addData("Shooter Actual", Shooter.speed)
    telemetry.addData("Shooter Target", Shooter.targetSpeed)
    Turret.IMUDegrees = PedroComponent.follower.pose.heading.rad.inDeg

    var pixelOffset = 0.0
    var hasLock = false

    if (alliance == Alliance.RED) {
      targetAprilTag = BotConstants.RED_ALLIANCE_APRILTAG_ID
    }
    if (alliance == Alliance.BLUE) {
      targetAprilTag = BotConstants.BLUE_ALLIANCE_APRILTAG_ID
    }
    if (aprilTag.detections.isNotEmpty()) {
      val desiredId = alliance.aprilTagId()
      for (detection in aprilTag.detections) {
        if (detection.id == desiredId) {
          hasLock = true
          pixelOffset = detection.center.x - (BotConstants.CAMERA_RESOLUTION_WIDTH / 2.0)
          telemetry.addData("pose", detection.ftcPose?.range ?: "null")
        }
      }
    }

    targetPose = alliance.targetPose()
    val offsetX = targetPose.x - (144.0 - PedroComponent.follower.pose.x)
    val offsetY = targetPose.y - (144.0 - PedroComponent.follower.pose.y)
    val goalAngle =
        atan2(
                offsetY,
                offsetX,
            )
            .rad
    telemetry.addData("Has Lock", hasLock)
    if (hasLock) {
      lastLockTime = System.currentTimeMillis()
      Turret.cameraTrackPower(pixelOffset).schedule()
    } else if (System.currentTimeMillis() - lastLockTime > LOCK_BUFFER_MS) {
      Turret.setAngle(
              goalAngle,
              true,
          )
          .schedule()
    } else {
      Turret.cameraTrackPower(0.0).schedule()
    }
    telemetry.addData("Relative X", offsetX)
    telemetry.addData("Relative Y", offsetY)
    BindingManager.update()
    telemetry.update()
  }

  override fun onStop() {
    BindingManager.reset()
  }
}

/**
 * Converts RGB color values to HSV.
 *
 * @param rIn Red component, in the range [0.0, 1.0].
 * @param gIn Green component, in the range [0.0, 1.0].
 * @param bIn Blue component, in the range [0.0, 1.0].
 * @return A FloatArray of size 3: [hue (0..360), saturation (0..255), value (0..255)].
 *
 * The conversion uses the standard RGB to HSV algorithm. Hue is in degrees
 * [0, 360), saturation and value are scaled to [0, 255].
 */
private fun rgbToHsv(rIn: Float, gIn: Float, bIn: Float): FloatArray {
  val r = rIn.coerceIn(teleop.HSV_CLAMP_MIN, teleop.HSV_CLAMP_MAX)
  val g = gIn.coerceIn(teleop.HSV_CLAMP_MIN, teleop.HSV_CLAMP_MAX)
  val b = bIn.coerceIn(teleop.HSV_CLAMP_MIN, teleop.HSV_CLAMP_MAX)

  val max = maxOf(r, g, b)
  val min = minOf(r, g, b)
  val delta = max - min

  val v = max
  val s = if (max <= teleop.HSV_CLAMP_MIN) teleop.HSV_CLAMP_MIN else delta / max

  var h = teleop.HSV_CLAMP_MIN
  if (delta > teleop.HSV_DELTA_THRESHOLD) {
    h =
        when (max) {
          r -> ((g - b) / delta) % teleop.HSV_HUE_SEGMENTS
          g -> ((b - r) / delta) + teleop.HSV_HUE_GREEN_OFFSET
          else -> ((r - g) / delta) + teleop.HSV_HUE_BLUE_OFFSET
        }
    h *= teleop.HSV_HUE_MULTIPLIER
    if (h < teleop.HSV_CLAMP_MIN) h += teleop.HSV_HUE_MAX
  }

  val sScaled = (s * teleop.HSV_SCALE_MAX).coerceIn(teleop.HSV_CLAMP_MIN, teleop.HSV_SCALE_MAX)
  val vScaled = (v * teleop.HSV_SCALE_MAX).coerceIn(teleop.HSV_CLAMP_MIN, teleop.HSV_SCALE_MAX)

  return floatArrayOf(h, sScaled, vScaled)
}
