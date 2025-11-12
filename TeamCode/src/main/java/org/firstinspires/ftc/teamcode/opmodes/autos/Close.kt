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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Indexer
import org.firstinspires.ftc.teamcode.subsystems.Lights
import org.firstinspires.ftc.teamcode.subsystems.LightsState
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret

@Autonomous
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

  val redStart = Pose(111.1, 133.3, 0.0)
  val blueStart = Pose(0.0, 0.0, 0.0)
  val redShoot = Pose(87.0, 82.0, 0.0)
  val redSpikeOneEnd = Pose(125.0, 85.0, 0.0)
  val redSpikeTwoStart = Pose(100.0, 61.0, 0.0)
  val redSpikeTwoEnd = Pose(130.0, 61.0, 0.0)
  val redGateHover = Pose(115.0, 70.0, 0.0)

  lateinit var redStartToShoot: PathChain
  lateinit var redShootToSpikeOne: PathChain
  lateinit var redSpikeOneToShoot: PathChain
  lateinit var redShootToSpikeTwo: PathChain
  lateinit var redSpikeTwoIntake: PathChain
  lateinit var redSpikeTwoToShoot: PathChain
  lateinit var redSpikeToGateHover: PathChain

  val waitForFeeder = 200.milliseconds
  val waitForIndexer = 750.milliseconds

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
            Delay(100.milliseconds),
            Indexer.indexerToSlot(1),
            Delay(100.milliseconds),
            Indexer.latchDown(),
            Delay(500.milliseconds),
            Indexer.waitForSlotBreakbeam(),
            Indexer.latchUp(),
            Delay(100.milliseconds),
            Indexer.indexerToSlot(2),
            Delay(100.milliseconds),
            Indexer.latchDown(),
            Delay(500.milliseconds),
            Indexer.waitForSlotBreakbeam(),
            Indexer.latchUp(),
        )

  val routine: Command
    get() =
        SequentialGroup(
            Indexer.latchUp(),
            Shooter.setSpeed(2_100.0),
            Turret.setAngle((-45).deg),
            FollowPath(redStartToShoot, false, 1.0),
            shootAll,
            Indexer.indexerToSlot(0),
            Indexer.setIntakePower(1.0),
            ParallelGroup(
                intakeAll,
                FollowPath(redShootToSpikeOne, false, 0.25),
            ),
            Indexer.indexerToSlot(0),
            FollowPath(redSpikeOneToShoot, false, 1.0),
            shootAll,
            Indexer.indexerToSlot(0),
            FollowPath(redShootToSpikeTwo, false, 1.0),
            ParallelGroup(
                intakeAll,
                FollowPath(redSpikeTwoIntake, false, 0.25),
            ),
            Indexer.indexerToSlot(0),
            FollowPath(redSpikeTwoToShoot, false, 1.0),
            shootAll,
            Indexer.setIntakePower(0.0),
            FollowPath(redSpikeToGateHover, true, 1.0),
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
                Path(BezierCurve(redSpikeTwoEnd, Pose(redShoot.x, redSpikeTwoEnd.y, 0.0), redShoot))
            )
            .setConstantHeadingInterpolation(0.0)
            .build()

    redSpikeToGateHover =
        PedroComponent.follower
            .pathBuilder()
            .addPath(Path(BezierLine(redSpikeTwoEnd, redGateHover)))
            .setConstantHeadingInterpolation(0.0)
            .build()

    Indexer.feed().schedule()
    Indexer.unFeed().schedule()
    Indexer.indexerToSlot(0).schedule()
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
