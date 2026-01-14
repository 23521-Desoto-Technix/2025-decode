package org.firstinspires.ftc.teamcode.opmodes.autos

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Tube
import org.firstinspires.ftc.teamcode.subsystems.Turret
import kotlin.time.Duration.Companion.milliseconds

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

  lateinit var startToShoot: PathChain
  lateinit var shootToSpike1: PathChain
  lateinit var spike1ToShoot: PathChain

  lateinit var routine: Command

  override fun onInit() {
    val intake = button { gamepad1.circle }.whenBecomesTrue { Tube.intakeAll.schedule() }
    Turret.setTargetAngle((-90).deg)
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

    routine =
        SequentialGroup(
            Flywheel.setSpeed(1_600.0),
            InstantCommand { Hood.position = 0.45 },
            InstantCommand { Turret.setTargetAngle((-92.5).deg) },
            FollowPath(startToShoot),
            Delay(500.milliseconds),
            Tube.shootAll(),
            Delay(750.milliseconds),
            FollowPath(shootToSpike1),
            Tube.intakeAll,
            FollowPath(spike1ToShoot),
            Tube.shootAll(),
        )
  }

  override fun onStartButtonPressed() {
    PedroComponent.follower.pose = redStart
    routine.schedule()
  }
}
