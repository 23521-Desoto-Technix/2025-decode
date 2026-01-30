package org.firstinspires.ftc.teamcode.opmodes.autos

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
import kotlin.time.Duration.Companion.milliseconds

@Autonomous(name = "Close 12", group = "Close", preselectTeleOp = "teleop")
class close12 : NextFTCOpMode() {
  init {
    addComponents(
        SubsystemComponent(Flywheel, Hood, Turret, Tube),
        BulkReadComponent,
        BindingsComponent,
        PedroComponent(Constants::createFollower),
    )
  }

  lateinit var routine: Command

  override fun onInit() {
    val intake = button { gamepad1.circle }.whenBecomesTrue { Tube.intakeAll.schedule() }
    Turret.setTargetAngle((-92.5).deg)
    telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
    telemetry.msTransmissionInterval = 25
  }

  private fun buildRoutine(paths: Map<String, PathChain>): Command {
    val turretAngle =
        when (BotState.alliance) {
          Alliance.RED -> (-92.5).deg
          Alliance.BLUE -> (95).deg
          else -> 0.0.deg
        }
    return SequentialGroup(
        Flywheel.setSpeed(1_600.0),
        InstantCommand { Hood.position = 0.45 },
        InstantCommand { Turret.setTargetAngle(turretAngle) },
        FollowPath(paths.getValue("startToShoot")),
        Delay(500.milliseconds),
        Tube.shootAll(),
        Delay(750.milliseconds),
        Tube.intakeAll,
        FollowPath(paths.getValue("shootToSpike2")),
        FollowPath(paths.getValue("spike2ToGate")),
        Delay(200.milliseconds),
        FollowPath(paths.getValue("spike2ToShoot")),
        Delay(500.milliseconds),
        Tube.shootAll(),
        Delay(750.milliseconds),
        Tube.intakeAll,
        FollowPath(paths.getValue("shootToSpike1")),
        FollowPath(paths.getValue("spike1ToShoot")),
        Delay(500.milliseconds),
        Tube.shootAll(),
        Delay(750.milliseconds),
        Tube.intakeAll,
        FollowPath(paths.getValue("shootToSpike3")),
        FollowPath(paths.getValue("spike3ToShoot")),
        Delay(750.milliseconds),
        Tube.shootAll(),
        Delay(500.milliseconds),
        FollowPath(paths.getValue("shootToPark")),
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
    val poses = AutoConstants.Poses.forAlliance(BotState.alliance)
    val paths = AutoConstants.Paths.forAlliance(BotState.alliance)
    routine = buildRoutine(paths)

    PedroComponent.follower.pose = poses.getValue("start")
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
