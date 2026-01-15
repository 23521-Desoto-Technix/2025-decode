package org.firstinspires.ftc.teamcode.opmodes.autos

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.bindings.BindingManager
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
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Tube
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.BotState
import org.firstinspires.ftc.teamcode.utils.PoseUtils.mirrorPose
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

  private data class PoseSet(
      val start: Pose,
      val shoot: Pose,
      val farShoot: Pose,
      val corner: Pose,
      val gate: Pose,
      val spike1: Pose,
      val spike2: Pose,
      val spike3: Pose,
  )

  private val redPoses =
      PoseSet(
          start = Pose(127.6, 120.8, -143.8.deg.inRad),
          shoot = Pose(85.0, 85.0, -45.0.deg.inRad),
          farShoot = Pose(85.0, 16.0, -90.0.deg.inRad),
          corner = Pose(130.0, 10.0, -20.0.deg.inRad),
          gate = Pose(125.0, 70.0, 0.0.deg.inRad),
          spike1 = Pose(125.0, 85.0, 0.0.deg.inRad),
          spike2 = Pose(125.0, 60.0, 0.0.deg.inRad),
          spike3 = Pose(125.0, 35.0, 0.0.deg.inRad),
      )

  private fun PoseSet.mirrored(): PoseSet {
    return PoseSet(
        start = mirrorPose(start),
        shoot = mirrorPose(shoot),
        farShoot = mirrorPose(farShoot),
        corner = mirrorPose(corner),
        gate = mirrorPose(gate),
        spike1 = mirrorPose(spike1),
        spike2 = mirrorPose(spike2),
        spike3 = mirrorPose(spike3),
    )
  }

  private fun posesForAlliance(): PoseSet {
    return when (BotState.alliance) {
      Alliance.BLUE -> redPoses.mirrored()
      else -> redPoses
    }
  }

  private data class AutoPaths(
      val startToShoot: PathChain,
      val shootToCorner: PathChain,
      val cornerToFarShoot: PathChain,
      val farShootToSpike2: PathChain,
      val spike2ToGate: PathChain,
      val gateToShoot: PathChain,
      val shootToSpike1: PathChain,
      val spike1ToShoot: PathChain,
      val shootToSpike3: PathChain,
      val spike3ToShoot: PathChain,
  )

  lateinit var routine: Command

  override fun onInit() {
    val intake = button { gamepad1.circle }.whenBecomesTrue { Tube.intakeAll.schedule() }
    Turret.setTargetAngle((-92.5).deg)
    telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
    telemetry.msTransmissionInterval = 25
  }

  private fun buildPaths(poses: PoseSet): AutoPaths {
    val shootSpike2Control = Pose(poses.shoot.x, poses.spike2.y - 5)
    val shootSpike3Control = Pose(poses.shoot.x, poses.spike3.y - 5)

    return AutoPaths(
        startToShoot =
            PedroComponent.follower
                .pathBuilder()
                .addPath(BezierLine(poses.start, poses.shoot))
                .setLinearHeadingInterpolation(poses.start.heading, poses.shoot.heading)
                .build(),
        shootToCorner =
            PedroComponent.follower
                .pathBuilder()
                .addPath(
                    BezierCurve(poses.shoot, Pose(poses.shoot.x, poses.corner.y), poses.corner)
                )
                .setConstantHeadingInterpolation(poses.corner.heading)
                .build(),
        cornerToFarShoot =
            PedroComponent.follower
                .pathBuilder()
                .addPath(BezierLine(poses.corner, poses.farShoot))
                .setLinearHeadingInterpolation(poses.corner.heading, poses.farShoot.heading)
                .build(),
        farShootToSpike2 =
            PedroComponent.follower
                .pathBuilder()
                .addPath(
                    BezierCurve(poses.farShoot, Pose(poses.farShoot.x, poses.gate.y), poses.gate)
                )
                .setConstantHeadingInterpolation(poses.spike2.heading)
                .build(),
        spike2ToGate =
            PedroComponent.follower
                .pathBuilder()
                .addPath(BezierCurve(poses.spike2, Pose(poses.spike2.x, poses.gate.y), poses.gate))
                .setConstantHeadingInterpolation(poses.gate.heading)
                .build(),
        gateToShoot =
            PedroComponent.follower
                .pathBuilder()
                .addPath(BezierCurve(poses.gate, Pose(poses.gate.x, poses.shoot.y), poses.shoot))
                .setLinearHeadingInterpolation(poses.gate.heading, poses.shoot.heading)
                .build(),
        shootToSpike1 =
            PedroComponent.follower
                .pathBuilder()
                .addPath(BezierLine(poses.shoot, poses.spike1))
                .setConstantHeadingInterpolation(poses.spike1.heading)
                .build(),
        spike1ToShoot =
            PedroComponent.follower
                .pathBuilder()
                .addPath(BezierLine(poses.spike1, poses.shoot))
                .setLinearHeadingInterpolation(poses.spike1.heading, poses.shoot.heading)
                .build(),
        shootToSpike3 =
            PedroComponent.follower
                .pathBuilder()
                .addPath(BezierCurve(poses.shoot, shootSpike3Control, poses.spike3))
                .setConstantHeadingInterpolation(poses.spike3.heading)
                .build(),
        spike3ToShoot =
            PedroComponent.follower
                .pathBuilder()
                .addPath(BezierLine(poses.spike3, poses.shoot))
                .setConstantHeadingInterpolation(poses.shoot.heading)
                .build(),
    )
  }

  private fun buildRoutine(paths: AutoPaths): Command {
    val turretAngle =
        when (BotState.alliance) {
          Alliance.RED -> (-92.5).deg
          Alliance.BLUE -> (92.5).deg
          else -> 0.0.deg
        }
    val farTurretAngle =
        when (BotState.alliance) {
          Alliance.RED -> (-20.0).deg
          Alliance.BLUE -> (20.0).deg
          else -> 0.0.deg
        }
    return SequentialGroup(
        Flywheel.setSpeed(1_600.0),
        InstantCommand { Hood.position = 0.45 },
        InstantCommand { Turret.setTargetAngle(turretAngle) },
        FollowPath(paths.startToShoot),
        Delay(500.milliseconds),
        Tube.shootAll(),
        Delay(750.milliseconds),
        Tube.intakeAll,
        Flywheel.setSpeed(2_050.0),
        InstantCommand { Hood.position = 0.7 },
        InstantCommand { Turret.setTargetAngle(farTurretAngle) },
        FollowPath(paths.shootToCorner),
        FollowPath(paths.cornerToFarShoot),
        Delay(500.milliseconds),
        Tube.shootAll(),
        Delay(750.milliseconds),
        Tube.intakeAll,
        FollowPath(paths.farShootToSpike2),
        //FollowPath(paths.spike2ToGate),
        Delay(500.milliseconds),
        FollowPath(paths.gateToShoot),
        Delay(500.milliseconds),
        Tube.shootAll(),
        Delay(750.milliseconds),
        Tube.intakeAll,
        FollowPath(paths.shootToSpike1),
        FollowPath(paths.spike1ToShoot),
        Delay(500.milliseconds),
        Tube.shootAll(),
        Delay(750.milliseconds),
        Tube.intakeAll,
        FollowPath(paths.shootToSpike3),
        FollowPath(paths.spike3ToShoot),
        Delay(500.milliseconds),
        Tube.shootAll(),
    )
  }

  override fun onWaitForStart() {

    val selectRed = button { gamepad1.circle }.whenBecomesTrue { BotState.alliance = Alliance.RED }
    val selectBlue = button { gamepad1.cross }.whenBecomesTrue { BotState.alliance = Alliance.BLUE }
    val allianceDisplay =
        when (BotState.alliance) {
          Alliance.RED ->
              "<span style=\"background-color: #FF0000; color: white;\">&nbsp;&nbsp;RED&nbsp;&nbsp;</span>"
          Alliance.BLUE ->
              "<span style=\"background-color: #0000FF; color: white;\">&nbsp;&nbsp;BLUE&nbsp;&nbsp;</span>"
          Alliance.UNKNOWN -> {

            if (((System.currentTimeMillis() / 500) % 2).toInt() == 0) {
              "<span style=\"background-color: yellow; color: black;\">&nbsp;&nbsp;!!&nbsp;&nbsp;UNKNOWN&nbsp;&nbsp;!!&nbsp;&nbsp;</span>"
            } else {
              "&nbsp;&nbsp;!!&nbsp;&nbsp;UNKNOWN&nbsp;&nbsp;!!&nbsp;&nbsp;"
            }
          }
        }

    telemetry.addLine(allianceDisplay)
    telemetry.addLine("RED: Circle ●")
    telemetry.addLine("BLUE: Cross ✕")

    BindingManager.update()
    telemetry.update()
  }

  override fun onStartButtonPressed() {
    val poses = posesForAlliance()
    val paths = buildPaths(poses)
    routine = buildRoutine(paths)

    PedroComponent.follower.pose = poses.start
    routine.schedule()
  }

  override fun onUpdate() {
    BotState.pose = PedroComponent.follower.pose
  }

  override fun onStop() {
    BotState.enabled = false
    Flywheel.setSpeed(0.0).schedule()
  }
}
