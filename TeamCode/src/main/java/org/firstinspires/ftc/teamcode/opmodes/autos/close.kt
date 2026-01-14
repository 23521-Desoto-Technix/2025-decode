package org.firstinspires.ftc.teamcode.opmodes.autos

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import kotlin.time.Duration.Companion.milliseconds
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Tube
import org.firstinspires.ftc.teamcode.subsystems.Turret

@Autonomous
class close : NextFTCOpMode() {
  init {
    addComponents(
        SubsystemComponent(Flywheel, Hood, Turret, Tube),
        BulkReadComponent,
        BindingsComponent,
        PedroComponent(Constants::createFollower),
    )
  }

  val redStart = Pose(127.6, 120.8, -143.8.deg.inRad)
  val redShoot = Pose(85.0, 85.0, -45.0.deg.inRad)
  val redSpike1 = Pose(125.0, 85.0, 0.0.deg.inRad)
  val redSpike2 = Pose(125.0, 60.0, 0.0.deg.inRad)
  val redSpike3 = Pose(125.0, 35.0, 0.0.deg.inRad)

  lateinit var startToShoot: PathChain
  lateinit var shootToSpike1: PathChain
  lateinit var spike1ToShoot: PathChain
  lateinit var shootToSpike2: PathChain
  lateinit var spike2ToShoot: PathChain
  lateinit var shootToSpike3: PathChain
  lateinit var spike3ToShoot: PathChain

  lateinit var routine: Command

  override fun onInit() {
    val intake = button { gamepad1.circle }.whenBecomesTrue { Tube.intakeAll.schedule() }
    Turret.setTargetAngle((-92.5).deg)
    startToShoot =
        PedroComponent.follower
            .pathBuilder()
            .addPath(BezierLine(redStart, redShoot))
            .setLinearHeadingInterpolation(redStart.heading, redShoot.heading)
            .build()

    shootToSpike1 =
        PedroComponent.follower
            .pathBuilder()
            .addPath(BezierLine(redShoot, redSpike1))
            .setConstantHeadingInterpolation(redSpike1.heading)
            .build()
    spike1ToShoot =
        PedroComponent.follower
            .pathBuilder()
            .addPath(BezierLine(redSpike1, redShoot))
            .setLinearHeadingInterpolation(redSpike1.heading, redShoot.heading)
            .build()
    shootToSpike2 =
        PedroComponent.follower
            .pathBuilder()
            .addPath(BezierCurve(redShoot, Pose(redShoot.x, redSpike2.y - 5), redSpike2))
            .setConstantHeadingInterpolation(redSpike2.heading)
            .build()
    spike2ToShoot =
        PedroComponent.follower
            .pathBuilder()
            .addPath(BezierLine(redSpike2, redShoot))
            .setConstantHeadingInterpolation(redShoot.heading)
            .build()
    shootToSpike3 =
        PedroComponent.follower
            .pathBuilder()
            .addPath(BezierCurve(redShoot, Pose(redShoot.x, redSpike3.y - 5), redSpike3))
            .setConstantHeadingInterpolation(redSpike3.heading)
            .build()
    spike3ToShoot =
        PedroComponent.follower
            .pathBuilder()
            .addPath(BezierLine(redSpike3, redShoot))
            .setConstantHeadingInterpolation(redShoot.heading)
            .build()

    routine =
        SequentialGroup(
            Flywheel.setSpeed(1_600.0),
            InstantCommand { Hood.position = 0.45 },
            InstantCommand { Turret.setTargetAngle((-92.5).deg) },
            FollowPath(startToShoot),
            Delay(500.milliseconds),
            Tube.shootAll(),
            Delay(750.milliseconds),
            Tube.intakeAll,
            FollowPath(shootToSpike1),
            FollowPath(spike1ToShoot),
            Delay(500.milliseconds),
            Tube.shootAll(),
            Delay(750.milliseconds),
            Tube.intakeAll,
            FollowPath(shootToSpike2),
            FollowPath(spike2ToShoot),
            Delay(500.milliseconds),
            Tube.shootAll(),
            Delay(750.milliseconds),
            Tube.intakeAll,
            FollowPath(shootToSpike3),
            FollowPath(spike3ToShoot),
            Tube.shootAll(),
            Delay(750.milliseconds),
        )
  }

  override fun onStartButtonPressed() {
    PedroComponent.follower.pose = redStart
    routine.schedule()
  }
}
