package org.firstinspires.ftc.teamcode.opmodes.autos

import android.util.Size
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.bindings.BindingManager
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import kotlin.math.atan2
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.opmodes.teleop
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Indexer
import org.firstinspires.ftc.teamcode.subsystems.Lights
import org.firstinspires.ftc.teamcode.subsystems.LightsState
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.time.Duration.Companion.milliseconds
import dev.nextftc.core.commands.delays.Delay

class ERCompat : NextFTCOpMode() {
  init {
    addComponents(
        SubsystemComponent(Shooter, Indexer, Lights, Turret, Hood),
        BulkReadComponent,
        BindingsComponent,
        PedroComponent(Constants::createFollower),
    )
  }

  enum class Alliance {
    RED,
    BLUE,
    UNKNOWN,
  }

  private lateinit var intakeBreakBeam: DigitalChannel
  private lateinit var leftBreakBeam: DigitalChannel
  private lateinit var rightBreakBeam: DigitalChannel

  lateinit var aprilTag: AprilTagProcessor
  lateinit var portal: VisionPortal

  val RESOLUTION_WIDTH: Int = 800
  val RESOLUTION_HEIGHT: Int = 600

  var targetPose = Pose(72.0, 144.0, 0.0)
  var targetAprilTag = 0

  var speedMultiplier = 1.0

  var alliance = teleop.Alliance.UNKNOWN

  val redStart = Pose(8.7, 64.5, 0.0)
  val blueStart = Pose(9.6, 81.7, 0.0)

  override fun onInit() {
    intakeBreakBeam = hardwareMap.get(DigitalChannel::class.java, "intakeBreakBeam")
    intakeBreakBeam.mode = DigitalChannel.Mode.INPUT
    leftBreakBeam = hardwareMap.get(DigitalChannel::class.java, "leftBreakBeam")
    leftBreakBeam.mode = DigitalChannel.Mode.INPUT
    rightBreakBeam = hardwareMap.get(DigitalChannel::class.java, "rightBreakBeam")
    rightBreakBeam.mode = DigitalChannel.Mode.INPUT
    Indexer.intakeBreakBeam = intakeBreakBeam
    Indexer.leftBreakBeam = leftBreakBeam
    Indexer.rightBreakBeam = rightBreakBeam
    Indexer.indexerToSlot(0).schedule()
    PedroComponent.follower.pose = Pose(72.0, 72.0, 0.0)
    PedroComponent.follower.breakFollowing()

    aprilTag =
        AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .setLensIntrinsics(667.154, 667.154, 438.702, 286.414)
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
            // ... these parameters are fx, fy, cx, cy.
            .build()

    portal =
        VisionPortal.Builder()
            .setCamera(hardwareMap.get<WebcamName?>(WebcamName::class.java, "turretCamera"))
            .setCameraResolution(Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(aprilTag)
            .build()
  }

  override fun onWaitForStart() {
    when (alliance) {
      teleop.Alliance.RED -> Lights.state = LightsState.ALLIANCE_RED
      teleop.Alliance.BLUE -> Lights.state = LightsState.ALLIANCE_BLUE
      teleop.Alliance.UNKNOWN -> Lights.state = LightsState.ALLIANCE_UNKNOWN
    }
    when (alliance) {
      teleop.Alliance.RED -> PedroComponent.follower.pose = redStart
      teleop.Alliance.BLUE -> PedroComponent.follower.pose = blueStart
      teleop.Alliance.UNKNOWN -> PedroComponent.follower.pose = redStart
    }
    if (gamepad1.circle) {
      alliance = teleop.Alliance.RED
    } else if (gamepad1.cross) {
      alliance = teleop.Alliance.BLUE
    }
  }

  override fun onUpdate() {

    var pixelOffset = 0.0
    var hasLock = false

    if (alliance == teleop.Alliance.RED) {
      targetAprilTag = 24
    }
    if (alliance == teleop.Alliance.BLUE) {
      targetAprilTag = 20
    }
    if (aprilTag.detections.isNotEmpty()) {
      for (detection in aprilTag.detections) {
        if (detection.id == targetAprilTag) {
          hasLock = true
          pixelOffset = detection.center.x - (RESOLUTION_WIDTH / 2.0)
          if (detection.ftcPose != null) {
            telemetry.addData("pose", detection.ftcPose.range)
          } else {
            telemetry.addData("pose", "null")
          }
        }
      }
    }

    if (alliance == teleop.Alliance.RED) {
      targetPose = Pose(144.0, 144.0, 0.0)
    } else if (alliance == teleop.Alliance.BLUE) {
      targetPose = Pose(144.0, 0.0, 0.0)
    }
    val offsetX = targetPose.x - PedroComponent.follower.pose.x
    val offsetY = targetPose.y - PedroComponent.follower.pose.y
    val goalAngle =
        atan2(
                offsetY,
                offsetX,
            )
            .rad
    telemetry.addData("Has Lock", hasLock)
    if (hasLock) {
      Turret.cameraTrackPower(pixelOffset).schedule()
    } else {
      Turret.setAngle(
              goalAngle,
              true,
          )
          .schedule()
    }
    BindingManager.update()
  }

  override fun onStartButtonPressed() {
    val waitForFeeder = 200.milliseconds
    val waitForIndexer = 500.milliseconds
    val shootAll =
      SequentialGroup(
        Indexer.feed(),
        Delay(waitForFeeder),
        Indexer.unFeed(),
        Delay(waitForFeeder),
        Indexer.toNextSlot(),
        Delay(waitForIndexer),

        Indexer.feed(),
        Delay(waitForFeeder),
        Indexer.unFeed(),
        Delay(waitForFeeder),
        Indexer.toNextSlot(),
        Delay(waitForIndexer),

        Indexer.feed(),
        Delay(waitForFeeder),
        Indexer.unFeed(),
        Delay(waitForFeeder),
        Indexer.toNextSlot(),
        Delay(waitForIndexer),
      )

    SequentialGroup(
      Shooter.setSpeed(2_300.0),
      Shooter.waitForSpeed(),
      shootAll,
    ).schedule()
  }
  override fun onStop() {
    BindingManager.reset()
  }
}
