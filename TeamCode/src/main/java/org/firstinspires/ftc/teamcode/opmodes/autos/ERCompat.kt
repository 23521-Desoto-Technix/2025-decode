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
import org.firstinspires.ftc.teamcode.Constants as AppConstants

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

  var targetPose = Pose(AppConstants.FIELD_CENTER_X, AppConstants.FIELD_MAX_Y, AppConstants.FIELD_MIN)
  var targetAprilTag = 0

  var alliance = Alliance.UNKNOWN

  val redStart = Pose(AppConstants.ER_RED_START_X, AppConstants.ER_RED_START_Y, AppConstants.FIELD_MIN)
  val redShoot = Pose(AppConstants.ER_RED_SHOOT_X, AppConstants.ER_RED_SHOOT_Y, AppConstants.FIELD_MIN)
  val blueStart = Pose(AppConstants.FIELD_MIN, AppConstants.FIELD_MIN, AppConstants.FIELD_MIN)
  val redSpikeOneStart = Pose(AppConstants.ER_RED_SPIKE_ONE_START_X, AppConstants.ER_RED_SPIKE_ONE_START_Y, AppConstants.FIELD_MIN)
  val redSpikeOneEnd = Pose(AppConstants.ER_RED_SPIKE_ONE_END_X, AppConstants.ER_RED_SPIKE_ONE_END_Y, AppConstants.FIELD_MIN)
  lateinit var startToRedSpikeOne: PathChain
  lateinit var redSpikeIntake: PathChain
  lateinit var redSpikeReturn: PathChain

  val waitForFeeder = AppConstants.WAIT_FOR_FEEDER_MS.milliseconds
  val waitForIndexer = AppConstants.WAIT_FOR_INDEXER_MS.milliseconds

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
            Indexer.indexerToSlot(AppConstants.INDEXER_SLOT_1),
            Delay(waitForIndexer),
            shoot,
            Indexer.indexerToSlot(AppConstants.INDEXER_SLOT_2),
            Delay(waitForIndexer),
            shoot,
        )

  val routine: Command
    get() =
        SequentialGroup(
            Indexer.latchUp(),
            Turret.setAngle(AppConstants.TURRET_ANGLE_ER_START.deg, true),
            Shooter.setSpeed(AppConstants.SHOOTER_SPEED_FAR),
            Shooter.waitForSpeed(),
            shootAll,
            Indexer.setIntakePower(AppConstants.INTAKE_POWER_FORWARD),
            Indexer.indexerToSlot(AppConstants.INDEXER_SLOT_0),
            FollowPath(startToRedSpikeOne, false, AppConstants.PATH_SPEED_FAST),
            ParallelGroup(
                SequentialGroup(
                    Indexer.latchDown(),
                    Indexer.waitForSlotBreakbeam(),
                    Indexer.latchUp(),
                    Delay(AppConstants.LATCH_WAIT_SHORT_MS.milliseconds),
                    Indexer.indexerToSlot(AppConstants.INDEXER_SLOT_1),
                    Delay(AppConstants.LATCH_WAIT_SHORT_MS.milliseconds),
                    Indexer.latchDown(),
                    Delay(AppConstants.LATCH_WAIT_LONG_MS.milliseconds),
                    Indexer.waitForSlotBreakbeam(),
                    Indexer.latchUp(),
                    Delay(AppConstants.LATCH_WAIT_SHORT_MS.milliseconds),
                    Indexer.indexerToSlot(AppConstants.INDEXER_SLOT_2),
                    Delay(AppConstants.LATCH_WAIT_SHORT_MS.milliseconds),
                    Indexer.latchDown(),
                    Delay(AppConstants.LATCH_WAIT_LONG_MS.milliseconds),
                    Indexer.waitForSlotBreakbeam(),
                    Indexer.latchUp(),
                ),
                FollowPath(redSpikeIntake, false, AppConstants.PATH_SPEED_MEDIUM),
            ),
            Indexer.setIntakePower(AppConstants.INTAKE_POWER_REVERSE),
            Indexer.indexerToSlot(AppConstants.INDEXER_SLOT_0),
            FollowPath(redSpikeReturn, false, AppConstants.PATH_SPEED_FAST),
            shootAll,
            Indexer.setIntakePower(AppConstants.INTAKE_POWER_OFF),
            InstantCommand { Lights.state = LightsState.ARTIFACT_GREEN },
            Delay(AppConstants.AUTO_END_WAIT_SECONDS.seconds),
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
    PedroComponent.follower.pose = Pose(AppConstants.FIELD_CENTER_X, AppConstants.FIELD_CENTER_Y, AppConstants.FIELD_MIN)
    PedroComponent.follower.breakFollowing()

    Indexer.feed().schedule()
    Indexer.unFeed().schedule()
    Indexer.indexerToSlot(AppConstants.INDEXER_SLOT_0).schedule()

    aprilTag =
        AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .setLensIntrinsics(AppConstants.CAMERA_LENS_FX, AppConstants.CAMERA_LENS_FY, AppConstants.CAMERA_LENS_CX, AppConstants.CAMERA_LENS_CY)
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
            // ... these parameters are fx, fy, cx, cy.
            .build()

    portal =
        VisionPortal.Builder()
            .setCamera(hardwareMap.get<WebcamName?>(WebcamName::class.java, "turretCamera"))
            .setCameraResolution(Size(AppConstants.CAMERA_RESOLUTION_WIDTH, AppConstants.CAMERA_RESOLUTION_HEIGHT))
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
      targetAprilTag = AppConstants.RED_ALLIANCE_APRILTAG_ID
    }
    if (alliance == Alliance.BLUE) {
      targetAprilTag = AppConstants.BLUE_ALLIANCE_APRILTAG_ID
    }
    if (aprilTag.detections.isNotEmpty()) {
      for (detection in aprilTag.detections) {
        if (detection.id == targetAprilTag) {
          hasLock = true
          pixelOffset = detection.center.x - (AppConstants.CAMERA_RESOLUTION_WIDTH / 2.0)
          if (detection.ftcPose != null) {
            telemetry.addData("pose", detection.ftcPose.range)
          } else {
            telemetry.addData("pose", "null")
          }
        }
      }
    }

    if (alliance == Alliance.RED) {
      targetPose = Pose(AppConstants.FIELD_MAX_X, AppConstants.FIELD_MAX_Y, AppConstants.FIELD_MIN)
    } else if (alliance == Alliance.BLUE) {
      targetPose = Pose(AppConstants.FIELD_MAX_X, AppConstants.FIELD_MIN, AppConstants.FIELD_MIN)
    }
    val offsetX = targetPose.x - (AppConstants.FIELD_MAX_X - PedroComponent.follower.pose.x)
    val offsetY = targetPose.y - (AppConstants.FIELD_MAX_Y - PedroComponent.follower.pose.y)
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
