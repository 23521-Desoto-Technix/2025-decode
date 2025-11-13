package org.firstinspires.ftc.teamcode.opmodes.autos

import android.util.Size
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.bindings.BindingManager
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
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
import kotlin.math.atan2
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import org.firstinspires.ftc.teamcode.BotConstants

@Autonomous(name = "East Rankin (6 far)")
class ERCompat : NextFTCOpMode() {
  init {
    addComponents(
        SubsystemComponent(Shooter, Indexer, Lights, Turret, Hood),
        BulkReadComponent,
        BindingsComponent,
        PedroComponent(Constants::createFollower),
    )
  }

  companion object {
    // Path following speeds
    const val PATH_SPEED_MEDIUM = 0.3
    const val PATH_SPEED_FAST = 1.0
    
    // Turret angle
    const val TURRET_ANGLE_START = 25.0
    
    // Autonomous end wait time (seconds)
    const val AUTO_END_WAIT_SECONDS = 30
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

  var targetPose = Pose(72.0, 144.0, 0.0)
  var targetAprilTag = 0

  var alliance = Alliance.UNKNOWN

  val redStart = Pose(80.1, 8.6, 0.0)
  val redShoot = Pose(82.0, 10.0, 0.0)
  val blueStart = Pose(0.0, 0.0, 0.0)
  val redSpikeOneStart = Pose(97.0, 35.0, 0.0)
  val redSpikeOneEnd = Pose(130.0, 35.0, 0.0)
  lateinit var startToRedSpikeOne: PathChain
  lateinit var redSpikeIntake: PathChain
  lateinit var redSpikeReturn: PathChain

  val waitForFeeder = BotConstants.WAIT_FOR_FEEDER_MS.milliseconds
  val waitForIndexer = BotConstants.WAIT_FOR_INDEXER_MS.milliseconds

  val shoot: Command
    get() =
        SequentialGroup(
            Indexer.feed(),
            Delay(waitForFeeder),
            Indexer.unFeed(),
            Delay(waitForFeeder),
        )

  val shootAll: Command
    get() =
        SequentialGroup(
            shoot,
            Indexer.indexerToSlot(BotConstants.INDEXER_SLOT_1),
            Delay(waitForIndexer),
            shoot,
            Indexer.indexerToSlot(BotConstants.INDEXER_SLOT_2),
            Delay(waitForIndexer),
            shoot,
        )

  val routine: Command
    get() =
        SequentialGroup(
            Indexer.latchUp(),
            Turret.setAngle(TURRET_ANGLE_START.deg, true),
            Shooter.setSpeed(BotConstants.SHOOTER_SPEED_FAR),
            Shooter.waitForSpeed(),
            shootAll,
            Indexer.setIntakePower(BotConstants.INTAKE_POWER_FORWARD),
            Indexer.indexerToSlot(BotConstants.INDEXER_SLOT_0),
            FollowPath(startToRedSpikeOne, false, PATH_SPEED_FAST),
            ParallelGroup(
                SequentialGroup(
                    Indexer.latchDown(),
                    Indexer.waitForSlotBreakbeam(),
                    Indexer.latchUp(),
                    Delay(BotConstants.LATCH_WAIT_SHORT_MS.milliseconds),
                    Indexer.indexerToSlot(BotConstants.INDEXER_SLOT_1),
                    Delay(BotConstants.LATCH_WAIT_SHORT_MS.milliseconds),
                    Indexer.latchDown(),
                    Delay(BotConstants.LATCH_WAIT_LONG_MS.milliseconds),
                    Indexer.waitForSlotBreakbeam(),
                    Indexer.latchUp(),
                    Delay(BotConstants.LATCH_WAIT_SHORT_MS.milliseconds),
                    Indexer.indexerToSlot(BotConstants.INDEXER_SLOT_2),
                    Delay(BotConstants.LATCH_WAIT_SHORT_MS.milliseconds),
                    Indexer.latchDown(),
                    Delay(BotConstants.LATCH_WAIT_LONG_MS.milliseconds),
                    Indexer.waitForSlotBreakbeam(),
                    Indexer.latchUp(),
                ),
                FollowPath(redSpikeIntake, false, PATH_SPEED_MEDIUM),
            ),
            Indexer.setIntakePower(BotConstants.INTAKE_POWER_REVERSE),
            Indexer.indexerToSlot(BotConstants.INDEXER_SLOT_0),
            FollowPath(redSpikeReturn, false, PATH_SPEED_FAST),
            shootAll,
            Indexer.setIntakePower(BotConstants.INTAKE_POWER_OFF),
            InstantCommand { Lights.state = LightsState.ARTIFACT_GREEN },
            Delay(AUTO_END_WAIT_SECONDS.seconds),
        )

  override fun onInit() {
    startToRedSpikeOne =
        PedroComponent.follower
            .pathBuilder()
            .addPath(Path(BezierLine(redStart, redSpikeOneStart)))
            .setConstantHeadingInterpolation(0.0)
            .build()
    redSpikeIntake =
        PedroComponent.follower
            .pathBuilder()
            .addPath(Path(BezierLine(redSpikeOneStart, redSpikeOneEnd)))
            .setConstantHeadingInterpolation(0.0)
            .build()
    redSpikeReturn =
        PedroComponent.follower
            .pathBuilder()
            .addPath(Path(BezierLine(redSpikeOneEnd, redShoot)))
            .setConstantHeadingInterpolation(0.0)
            .build()
    intakeBreakBeam = hardwareMap.get(DigitalChannel::class.java, "intakeBreakBeam")
    intakeBreakBeam.mode = DigitalChannel.Mode.INPUT
    leftBreakBeam = hardwareMap.get(DigitalChannel::class.java, "leftBreakBeam")
    leftBreakBeam.mode = DigitalChannel.Mode.INPUT
    rightBreakBeam = hardwareMap.get(DigitalChannel::class.java, "rightBreakBeam")
    rightBreakBeam.mode = DigitalChannel.Mode.INPUT
    Indexer.intakeBreakBeam = intakeBreakBeam
    Indexer.leftBreakBeam = leftBreakBeam
    Indexer.rightBreakBeam = rightBreakBeam
    PedroComponent.follower.pose = BotConstants.FIELD_CENTER
    PedroComponent.follower.breakFollowing()

    Indexer.feed().schedule()
    Indexer.unFeed().schedule()
    Indexer.indexerToSlot(BotConstants.INDEXER_SLOT_0).schedule()

    aprilTag =
        AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .setLensIntrinsics(BotConstants.CAMERA_LENS_FX, BotConstants.CAMERA_LENS_FY, BotConstants.CAMERA_LENS_CX, BotConstants.CAMERA_LENS_CY)
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
            // ... these parameters are fx, fy, cx, cy.
            .build()

    portal =
        VisionPortal.Builder()
            .setCamera(hardwareMap.get<WebcamName?>(WebcamName::class.java, "turretCamera"))
            .setCameraResolution(Size(BotConstants.CAMERA_RESOLUTION_WIDTH, BotConstants.CAMERA_RESOLUTION_HEIGHT))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(aprilTag)
            .build()
  }

  override fun onWaitForStart() {
    when (alliance) {
      Alliance.RED -> Lights.state = LightsState.ALLIANCE_RED
      Alliance.BLUE -> Lights.state = LightsState.ALLIANCE_BLUE
      Alliance.UNKNOWN -> Lights.state = LightsState.ALLIANCE_UNKNOWN
    }
    when (alliance) {
      Alliance.RED -> PedroComponent.follower.pose = redStart
      Alliance.BLUE -> PedroComponent.follower.pose = blueStart
      Alliance.UNKNOWN -> PedroComponent.follower.pose = redStart
    }
    if (gamepad1.circle) {
      alliance = Alliance.RED
    } else if (gamepad1.cross) {
      alliance = Alliance.BLUE
    }
  }

  override fun onUpdate() {

    var pixelOffset = 0.0
    var hasLock = false

    if (alliance == Alliance.RED) {
      targetAprilTag = BotConstants.RED_ALLIANCE_APRILTAG_ID
    }
    if (alliance == Alliance.BLUE) {
      targetAprilTag = BotConstants.BLUE_ALLIANCE_APRILTAG_ID
    }
    if (aprilTag.detections.isNotEmpty()) {
      for (detection in aprilTag.detections) {
        if (detection.id == targetAprilTag) {
          hasLock = true
          pixelOffset = detection.center.x - (BotConstants.CAMERA_RESOLUTION_WIDTH / 2.0)
          if (detection.ftcPose != null) {
            telemetry.addData("pose", detection.ftcPose.range)
          } else {
            telemetry.addData("pose", "null")
          }
        }
      }
    }

    if (alliance == Alliance.RED) {
      targetPose = BotConstants.RED_TARGET_POSE
    } else if (alliance == Alliance.BLUE) {
      targetPose = BotConstants.BLUE_TARGET_POSE
    }
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
      /*Turret.cameraTrackPower(pixelOffset).schedule()*/
    } else {
      /*Turret.setAngle(
          goalAngle,
          true,
      )
      .schedule()*/
    }
    BindingManager.update()
  }

  override fun onStartButtonPressed() {
    routine()
  }

  override fun onStop() {
    BindingManager.reset()
  }
}
