package org.firstinspires.ftc.teamcode.opmodes.autos

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
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
import kotlin.time.Duration.Companion.milliseconds
import org.firstinspires.ftc.teamcode.Constants as AppConstants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Indexer
import org.firstinspires.ftc.teamcode.subsystems.Lights
import org.firstinspires.ftc.teamcode.subsystems.LightsState
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret

@Autonomous(name = "Close (6 near)")
class Close : NextFTCOpMode() {
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

  var alliance = Alliance.UNKNOWN

  private lateinit var intakeBreakBeam: DigitalChannel
  private lateinit var leftBreakBeam: DigitalChannel
  private lateinit var rightBreakBeam: DigitalChannel

  val redStart = Pose(AppConstants.CLOSE_RED_START_X, AppConstants.CLOSE_RED_START_Y, AppConstants.FIELD_MIN)
  val blueStart = Pose(AppConstants.FIELD_MIN, AppConstants.FIELD_MIN, AppConstants.FIELD_MIN)
  val redShoot = Pose(AppConstants.CLOSE_RED_SHOOT_X, AppConstants.CLOSE_RED_SHOOT_Y, AppConstants.FIELD_MIN)
  val redSpikeOneEnd = Pose(AppConstants.CLOSE_RED_SPIKE_ONE_END_X, AppConstants.CLOSE_RED_SPIKE_ONE_END_Y, AppConstants.FIELD_MIN)
  val redSpikeTwoStart = Pose(AppConstants.CLOSE_RED_SPIKE_TWO_START_X, AppConstants.CLOSE_RED_SPIKE_TWO_START_Y, AppConstants.FIELD_MIN)
  val redSpikeTwoEnd = Pose(AppConstants.CLOSE_RED_SPIKE_TWO_END_X, AppConstants.CLOSE_RED_SPIKE_TWO_END_Y, AppConstants.FIELD_MIN)
  val redGateHover = Pose(AppConstants.CLOSE_RED_GATE_HOVER_X, AppConstants.CLOSE_RED_GATE_HOVER_Y, AppConstants.FIELD_MIN)

  lateinit var redStartToShoot: PathChain
  lateinit var redShootToSpikeOne: PathChain
  lateinit var redSpikeOneToShoot: PathChain
  lateinit var redShootToSpikeTwo: PathChain
  lateinit var redSpikeTwoIntake: PathChain
  lateinit var redSpikeTwoToShoot: PathChain
  lateinit var redSpikeToGateHover: PathChain

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

  val intakeAll: Command
    get() =
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
        )

  val routine: Command
    get() =
        SequentialGroup(
            Indexer.latchUp(),
            Shooter.setSpeed(AppConstants.SHOOTER_SPEED_CLOSE),
            Turret.setAngle(AppConstants.TURRET_ANGLE_CLOSE_START.deg),
            FollowPath(redStartToShoot, false, AppConstants.PATH_SPEED_FAST),
            shootAll,
            Indexer.indexerToSlot(AppConstants.INDEXER_SLOT_0),
            Indexer.setIntakePower(AppConstants.INTAKE_POWER_FORWARD),
            ParallelGroup(
                intakeAll,
                FollowPath(redShootToSpikeOne, false, AppConstants.PATH_SPEED_SLOW),
            ),
            Indexer.indexerToSlot(AppConstants.INDEXER_SLOT_0),
            FollowPath(redSpikeOneToShoot, false, AppConstants.PATH_SPEED_FAST),
            shootAll,
            Indexer.indexerToSlot(AppConstants.INDEXER_SLOT_0),
            FollowPath(redShootToSpikeTwo, false, AppConstants.PATH_SPEED_FAST),
            ParallelGroup(
                intakeAll,
                FollowPath(redSpikeTwoIntake, false, AppConstants.PATH_SPEED_SLOW),
            ),
            Indexer.indexerToSlot(AppConstants.INDEXER_SLOT_0),
            FollowPath(redSpikeTwoToShoot, false, AppConstants.PATH_SPEED_FAST),
            shootAll,
            Indexer.setIntakePower(AppConstants.INTAKE_POWER_OFF),
            FollowPath(redSpikeToGateHover, true, AppConstants.PATH_SPEED_FAST),
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

    redStartToShoot =
        PedroComponent.follower
            .pathBuilder()
            .addPath(Path(BezierLine(redStart, redShoot)))
            .setConstantHeadingInterpolation(0.0)
            .build()

    redShootToSpikeOne =
        PedroComponent.follower
            .pathBuilder()
            .addPath(Path(BezierLine(redShoot, redSpikeOneEnd)))
            .setConstantHeadingInterpolation(0.0)
            .build()

    redSpikeOneToShoot =
        PedroComponent.follower
            .pathBuilder()
            .addPath(Path(BezierLine(redSpikeOneEnd, redShoot)))
            .setConstantHeadingInterpolation(0.0)
            .build()

    redShootToSpikeTwo =
        PedroComponent.follower
            .pathBuilder()
            .addPath(Path(BezierLine(redShoot, redSpikeTwoStart)))
            .setConstantHeadingInterpolation(0.0)
            .build()

    redSpikeTwoIntake =
        PedroComponent.follower
            .pathBuilder()
            .addPath(Path(BezierLine(redSpikeTwoStart, redSpikeTwoEnd)))
            .setConstantHeadingInterpolation(0.0)
            .build()

    redSpikeTwoToShoot =
        PedroComponent.follower
            .pathBuilder()
            .addPath(
                Path(BezierCurve(redSpikeTwoEnd, Pose(redShoot.x, redSpikeTwoEnd.y, AppConstants.FIELD_MIN), redShoot))
            )
            .setConstantHeadingInterpolation(AppConstants.FIELD_MIN)
            .build()

    redSpikeToGateHover =
        PedroComponent.follower
            .pathBuilder()
            .addPath(Path(BezierLine(redSpikeTwoEnd, redGateHover)))
            .setConstantHeadingInterpolation(0.0)
            .build()

    Indexer.feed().schedule()
    Indexer.unFeed().schedule()
    Indexer.indexerToSlot(AppConstants.INDEXER_SLOT_0).schedule()
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

  override fun onStartButtonPressed() {
    routine()
  }
}
