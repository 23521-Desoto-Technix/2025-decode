package org.firstinspires.ftc.teamcode.opmodes.autos

import android.util.Size
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
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
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.Motif
import org.firstinspires.ftc.teamcode.utils.PoseUtils
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.time.Duration.Companion.milliseconds

@Autonomous(name = "Close (9 non sorted)")
class Close : NextFTCOpMode() {
  init {
    addComponents(
        SubsystemComponent(Shooter, Indexer, Lights, Turret, Hood),
        BulkReadComponent,
        BindingsComponent,
        PedroComponent(Constants::createFollower),
    )
  }

  companion object {
    const val PATH_SPEED_SLOW = 0.25
    const val PATH_SPEED_FAST = 1.0

    const val TURRET_ANGLE_RED_UNKNOWN = -100.0
    const val TURRET_ANGLE_BLUE_UNKNOWN = 0.0

    const val TURRET_ANGLE_RED_KNOWN = -45.0
    const val TURRET_ANGLE_BLUE_KNOWN = 45.0
  }

  var alliance = Alliance.UNKNOWN
  var motif = Motif.UNKNOWN

  private lateinit var intakeBreakBeam: DigitalChannel
  private lateinit var leftBreakBeam: DigitalChannel
  private lateinit var rightBreakBeam: DigitalChannel
  private lateinit var aprilTag: AprilTagProcessor
  private lateinit var portal: VisionPortal

  val redStart = Pose(113.0, 131.0, 0.0)
  val redShoot = Pose(87.0, 82.0, 0.0)
  val redSpikeOneEnd = Pose(125.0, 82.0, 0.0)
  val redSpikeTwoStart = Pose(100.0, 58.0, 0.0)
  val redSpikeTwoEnd = Pose(130.0, 58.0, 0.0)
  val redGateHover = Pose(115.0, 70.0, 0.0)

  var turretAngle = 0.0

  val blueStart: Pose
    get() = PoseUtils.mirrorPose(redStart)

  val blueShoot: Pose
    get() = PoseUtils.mirrorPose(redShoot)

  val blueSpikeOneEnd: Pose
    get() = PoseUtils.mirrorPose(redSpikeOneEnd)

  val blueSpikeTwoStart: Pose
    get() = PoseUtils.mirrorPose(redSpikeTwoStart)

  val blueSpikeTwoEnd: Pose
    get() = PoseUtils.mirrorPose(redSpikeTwoEnd)

  val blueGateHover: Pose
    get() = PoseUtils.mirrorPose(redGateHover)

  val currentStart: Pose
    get() = if (alliance == Alliance.RED) redStart else blueStart

  val currentShoot: Pose
    get() = if (alliance == Alliance.RED) redShoot else blueShoot

  val currentSpikeOneEnd: Pose
    get() = if (alliance == Alliance.RED) redSpikeOneEnd else blueSpikeOneEnd

  val currentSpikeTwoStart: Pose
    get() = if (alliance == Alliance.RED) redSpikeTwoStart else blueSpikeTwoStart

  val currentSpikeTwoEnd: Pose
    get() = if (alliance == Alliance.RED) redSpikeTwoEnd else blueSpikeTwoEnd

  val currentGateHover: Pose
    get() = if (alliance == Alliance.RED) redGateHover else blueGateHover

  lateinit var startToShoot: PathChain
  lateinit var shootToSpikeOne: PathChain
  lateinit var spikeOneToShoot: PathChain
  lateinit var shootToSpikeTwo: PathChain
  lateinit var spikeTwoIntake: PathChain
  lateinit var spikeTwoToShoot: PathChain
  lateinit var spikeToGateHover: PathChain

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

  private fun buildThreeShotSequence(vararg slots: Int): Command {
    require(slots.size == 3) { "Expected exactly 3 slots" }
    val commands = mutableListOf<Command>()
    for (i in slots.indices) {
      commands.add(Indexer.indexerToSlot(slots[i]))
      commands.add(Delay(waitForIndexer))
      commands.add(shoot)
    }
    return SequentialGroup(*commands.toTypedArray())
  }

  val shootAll0: Command
    get() = buildThreeShotSequence(0, 1, 2)

  val shootAll1: Command
    get() = buildThreeShotSequence(1, 0, -1)

  val shootAll2: Command
    get() = buildThreeShotSequence(0, -1, -2)

  private fun shootSequenceForSeries(seriesIndex: Int): Command {
    return when (motif) {
      Motif.PPG ->
          when (seriesIndex) {
            0 -> shootAll1
            1 -> shootAll1
            else -> shootAll2
          }
      Motif.PGP ->
          when (seriesIndex) {
            0 -> shootAll1
            1 -> shootAll0
            else -> shootAll2
          }
      Motif.GPP ->
          when (seriesIndex) {
            0 -> shootAll0
            1 -> shootAll0
            else -> shootAll1
          }
      Motif.UNKNOWN ->
          when (seriesIndex) {
            0 -> shootAll0
            1 -> shootAll0
            else -> shootAll0
          }
    }
  }

  val shootAll: Command
    get() =
        SequentialGroup(
            shoot,
            Indexer.indexerToSlot(1),
            Delay(waitForIndexer),
            shoot,
            Indexer.indexerToSlot(2),
            Delay(waitForIndexer),
            shoot,
        )

  val intakeAll: Command
    get() =
        SequentialGroup(
            Indexer.latchDown(),
            Indexer.waitForSlotBreakbeam(),
            Indexer.latchUp(),
            Delay(BotConstants.LATCH_WAIT_SHORT_MS.milliseconds),
            Indexer.indexerToSlot(1),
            Delay(BotConstants.LATCH_WAIT_SHORT_MS.milliseconds),
            Indexer.latchDown(),
            Delay(BotConstants.LATCH_WAIT_LONG_MS.milliseconds),
            Indexer.waitForSlotBreakbeam(),
            Indexer.latchUp(),
            Delay(BotConstants.LATCH_WAIT_SHORT_MS.milliseconds),
            Indexer.indexerToSlot(2),
            Delay(BotConstants.LATCH_WAIT_SHORT_MS.milliseconds),
            Indexer.latchDown(),
            Delay(BotConstants.LATCH_WAIT_LONG_MS.milliseconds),
            Indexer.waitForSlotBreakbeam(),
            Indexer.latchUp(),
        )

  val routine: Command
    get() =
        SequentialGroup(
            Indexer.latchUp(),
            Shooter.setSpeed(BotConstants.SHOOTER_SPEED_CLOSE),
            FollowPath(startToShoot, false, PATH_SPEED_FAST),
            shootSequenceForSeries(0),
            Shooter.setSpeed(BotConstants.SHOOTER_SPEED_OFF),
            Indexer.indexerToSlot(0),
            Indexer.setIntakePower(BotConstants.INTAKE_POWER_FORWARD),
            ParallelGroup(
                intakeAll,
                FollowPath(shootToSpikeOne, false, PATH_SPEED_SLOW),
            ),
            Shooter.setSpeed(BotConstants.SHOOTER_SPEED_CLOSE),
            Indexer.indexerToSlot(0),
            FollowPath(spikeOneToShoot, false, PATH_SPEED_FAST),
            shootSequenceForSeries(1),
            Shooter.setSpeed(BotConstants.SHOOTER_SPEED_OFF),
            Indexer.indexerToSlot(0),
            FollowPath(shootToSpikeTwo, false, PATH_SPEED_FAST),
            ParallelGroup(
                intakeAll,
                FollowPath(spikeTwoIntake, false, PATH_SPEED_SLOW),
            ),
            Shooter.setSpeed(BotConstants.SHOOTER_SPEED_CLOSE),
            Indexer.indexerToSlot(0),
            FollowPath(spikeTwoToShoot, false, PATH_SPEED_FAST),
            shootSequenceForSeries(2),
            Indexer.setIntakePower(BotConstants.INTAKE_POWER_OFF),
            Shooter.disable(),
            Turret.disable(),
            Indexer.disable(),
            FollowPath(spikeToGateHover, true, PATH_SPEED_FAST),
        )

  override fun onInit() {
    intakeBreakBeam = hardwareMap.get(DigitalChannel::class.java, "intakeBreakBeam")
    leftBreakBeam = hardwareMap.get(DigitalChannel::class.java, "leftBreakBeam")
    rightBreakBeam = hardwareMap.get(DigitalChannel::class.java, "rightBreakBeam")
    intakeBreakBeam.mode = DigitalChannel.Mode.INPUT
    leftBreakBeam.mode = DigitalChannel.Mode.INPUT
    rightBreakBeam.mode = DigitalChannel.Mode.INPUT

    Indexer.intakeBreakBeam = intakeBreakBeam
    Indexer.leftBreakBeam = leftBreakBeam
    Indexer.rightBreakBeam = rightBreakBeam

    initializePaths()

    Indexer.feed().schedule()
    Indexer.unFeed().schedule()
    Indexer.indexerToSlot(0).schedule()

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

  private fun initializePaths() {
    startToShoot = PoseUtils.createBasicPath(currentStart, currentShoot)
    shootToSpikeOne = PoseUtils.createBasicPath(currentShoot, currentSpikeOneEnd)
    spikeOneToShoot = PoseUtils.createBasicPath(currentSpikeOneEnd, currentShoot)
    shootToSpikeTwo = PoseUtils.createBasicPath(currentShoot, currentSpikeTwoStart)
    spikeTwoIntake = PoseUtils.createBasicPath(currentSpikeTwoStart, currentSpikeTwoEnd)
    spikeTwoToShoot =
        PoseUtils.createCurvedPath(
            currentSpikeTwoEnd,
            Pose(currentShoot.x, currentSpikeTwoEnd.y, 0.0),
            currentShoot,
        )
    spikeToGateHover = PoseUtils.createBasicPath(currentSpikeTwoEnd, currentGateHover)
  }

  override fun onWaitForStart() {
    if (gamepad1.circle) {
      alliance = Alliance.RED
    } else if (gamepad1.cross) {
      alliance = Alliance.BLUE
    }

    Lights.state = alliance.lightsState()

    if (alliance.isKnown) {
      initializePaths()
    }

    PedroComponent.follower.pose = if (alliance == Alliance.BLUE) blueStart else redStart

    turretAngle =
        when (alliance) {
          Alliance.RED -> TURRET_ANGLE_RED_UNKNOWN
          Alliance.BLUE -> TURRET_ANGLE_BLUE_UNKNOWN
          Alliance.UNKNOWN -> 0.0
        }
  }

  override fun onUpdate() {
    blackboard["pose"] = PedroComponent.follower.pose
    blackboard["alliance"] = alliance.toString()
    blackboard["motif"] = motif.toString()

    if (this::aprilTag.isInitialized) {
      if (aprilTag.detections.isNotEmpty()) {
        var foundKnownMotif: Motif? = null
        for (detection in aprilTag.detections) {
          val detectedMotif = Motif.fromAprilTagId(detection.id)
          if (detectedMotif.isKnown) {
            foundKnownMotif = detectedMotif
          }

          telemetry.addData("AprilTag ID", detection.id)
          telemetry.addData(
              "AprilTag Center",
              "x=%.1f y=%.1f".format(detection.center.x, detection.center.y),
          )
          telemetry.addData("AprilTag Pose", detection.ftcPose?.range ?: "null")
        }

        if (foundKnownMotif != null) {
          motif = foundKnownMotif
        }

        telemetry.addData("Motif", motif.toString())
      } else {
        telemetry.addData("AprilTag", "no detections")
      }
      telemetry.addData("Motif", motif.toString())
      telemetry.update()
    }

    turretAngle =
        if (motif.isKnown) {
          when (alliance) {
            Alliance.RED -> TURRET_ANGLE_RED_KNOWN
            Alliance.BLUE -> TURRET_ANGLE_BLUE_KNOWN
            Alliance.UNKNOWN -> 0.0
          }
        } else {
          when (alliance) {
            Alliance.RED -> TURRET_ANGLE_RED_UNKNOWN
            Alliance.BLUE -> TURRET_ANGLE_BLUE_UNKNOWN
            Alliance.UNKNOWN -> 0.0
          }
        }
    Turret.setAngle(turretAngle.deg).schedule()
  }

  override fun onStartButtonPressed() {
    routine()
  }
}
