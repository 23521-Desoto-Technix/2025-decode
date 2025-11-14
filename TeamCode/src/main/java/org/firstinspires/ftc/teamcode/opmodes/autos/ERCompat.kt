package org.firstinspires.ftc.teamcode.opmodes.autos

import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.BotConstants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Indexer
import org.firstinspires.ftc.teamcode.subsystems.Lights
import org.firstinspires.ftc.teamcode.subsystems.LightsState
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.utils.PoseUtils
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

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
    const val PATH_SPEED_SLOW = 0.25
    const val PATH_SPEED_FAST = 1.0

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

  var alliance = Alliance.UNKNOWN
  var turretAngle = 0.0

  val redStart = Pose(80.1, 8.6, 0.0)
  val redShoot = Pose(82.0, 10.0, 0.0)
  val redSpikeOneStart = Pose(97.0, 35.0, 0.0)
  val redSpikeOneEnd = Pose(130.0, 35.0, 0.0)
  val redPark = Pose(105.0, 30.0, 0.0)

  var startDelay = 0.seconds
  var secondaryDelay = 0.seconds

  val blueStart: Pose
    get() = PoseUtils.mirrorPose(redStart)

  val blueShoot: Pose
    get() = PoseUtils.mirrorPose(redShoot)

  val blueSpikeOneStart: Pose
    get() = PoseUtils.mirrorPose(redSpikeOneStart)

  val blueSpikeOneEnd: Pose
    get() = PoseUtils.mirrorPose(redSpikeOneEnd)

  val bluePark: Pose
    get() = PoseUtils.mirrorPose(redPark)

  val currentStart: Pose
    get() = if (alliance == Alliance.RED) redStart else blueStart

  val currentShoot: Pose
    get() = if (alliance == Alliance.RED) redShoot else blueShoot

  val currentSpikeOneStart: Pose
    get() = if (alliance == Alliance.RED) redSpikeOneStart else blueSpikeOneStart

  val currentSpikeOneEnd: Pose
    get() = if (alliance == Alliance.RED) redSpikeOneEnd else blueSpikeOneEnd

  val currentPark: Pose
    get() = if (alliance == Alliance.RED) redPark else bluePark

  lateinit var startToRedSpikeOne: PathChain
  lateinit var redSpikeIntake: PathChain
  lateinit var redSpikeReturn: PathChain
  lateinit var redShootToPark: PathChain

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
            Turret.setAngle(turretAngle.deg, true),
            Shooter.setSpeed(BotConstants.SHOOTER_SPEED_FAR),
            Shooter.waitForSpeed(),
            Delay(startDelay),
            shootAll,
            Shooter.setSpeed(BotConstants.SHOOTER_SPEED_OFF),
            Indexer.setIntakePower(BotConstants.INTAKE_POWER_FORWARD),
            Indexer.indexerToSlot(0),
            FollowPath(startToRedSpikeOne, false, PATH_SPEED_FAST),
            ParallelGroup(
                intakeAll,
                FollowPath(redSpikeIntake, false, PATH_SPEED_SLOW),
            ),
            Shooter.setSpeed(BotConstants.SHOOTER_SPEED_FAR),
            Indexer.indexerToSlot(0),
            FollowPath(redSpikeReturn, false, PATH_SPEED_FAST),
            Delay(secondaryDelay),
            shootAll,
            Indexer.setIntakePower(BotConstants.INTAKE_POWER_OFF),
            InstantCommand { Lights.state = LightsState.ARTIFACT_GREEN },
            Shooter.disable(),
            Turret.disable(),
            Indexer.disable(),
            FollowPath(redShootToPark, false, PATH_SPEED_FAST),
            Delay(AUTO_END_WAIT_SECONDS.seconds),
        )

  private fun initializePaths() {
    startToRedSpikeOne = PoseUtils.createBasicPath(currentStart, currentSpikeOneStart)
    redSpikeIntake = PoseUtils.createBasicPath(currentSpikeOneStart, currentSpikeOneEnd)
    redSpikeReturn = PoseUtils.createBasicPath(currentSpikeOneEnd, currentShoot)
    redShootToPark = PoseUtils.createBasicPath(currentShoot, currentPark)
  }

  override fun onInit() {
    val startDelayUp =
        button { gamepad1.dpad_up }
            .inLayer("init") {
              startDelay = (startDelay + 500.milliseconds).coerceAtMost(15.seconds)
            }
    val startDelayDown =
        button { gamepad1.dpad_down }
            .inLayer("init") {
              startDelay = (startDelay - 500.milliseconds).coerceAtLeast(0.seconds)
            }
    val secondaryDelayUp =
        button { gamepad1.triangle }
            .inLayer("init") {
              secondaryDelay = (secondaryDelay + 500.milliseconds).coerceAtMost(15.seconds)
            }
    val secondaryDelayDown =
        button { gamepad1.cross }
            .inLayer("init") {
              secondaryDelay = (secondaryDelay - 500.milliseconds).coerceAtLeast(0.seconds)
            }
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
    Indexer.indexerToSlot(0).schedule()

    initializePaths()
  }

  override fun onWaitForStart() {
    if (gamepad1.circle) {
      alliance = Alliance.RED
    } else if (gamepad1.cross) {
      alliance = Alliance.BLUE
    }

    when (alliance) {
      Alliance.RED -> Lights.state = LightsState.ALLIANCE_RED
      Alliance.BLUE -> Lights.state = LightsState.ALLIANCE_BLUE
      Alliance.UNKNOWN -> Lights.state = LightsState.ALLIANCE_UNKNOWN
    }

    if (alliance != Alliance.UNKNOWN) {
      initializePaths()
    }

    when (alliance) {
      Alliance.RED -> PedroComponent.follower.pose = redStart
      Alliance.BLUE -> PedroComponent.follower.pose = blueStart
      Alliance.UNKNOWN -> PedroComponent.follower.pose = redStart
    }

    turretAngle =
        when (alliance) {
          Alliance.RED -> 25.0
          Alliance.BLUE -> 160.0
          Alliance.UNKNOWN -> 0.0
        }
  }

  override fun onUpdate() {

    blackboard["pose"] = PedroComponent.follower.pose
    blackboard["alliance"] = alliance.toString()
    telemetry.addLine("Increase: Up, Decrease: Down")
    telemetry.addData("Start Delay", startDelay.inWholeMilliseconds)
    telemetry.addLine("Increase: Triangle, Decrease: Cross")
    telemetry.addData("Secondary Delay", secondaryDelay.inWholeMilliseconds)
    telemetry.update()
    BindingManager.update()
  }

  override fun onStartButtonPressed() {
    routine()
  }

  override fun onStop() {
    BindingManager.reset()
  }
}
