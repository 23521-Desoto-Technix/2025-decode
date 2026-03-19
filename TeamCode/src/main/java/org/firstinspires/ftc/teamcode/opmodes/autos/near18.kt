package org.firstinspires.ftc.teamcode.opmodes.autos

import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.lynx.LynxModule
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
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.TelemetryImplUpstreamSubmission
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Tube
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.BotState
import kotlin.time.Duration.Companion.milliseconds

@Autonomous(name = "Near 18", group = "Near", preselectTeleOp = "teleop")
class near18 : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(Flywheel, Hood, Turret, Tube),
            // BulkReadComponent,
            BindingsComponent,
            PedroComponent(Constants::createFollower),
        )
        telemetry = TelemetryImplUpstreamSubmission(this)
    }

    lateinit var routine: Command

    private lateinit var allHubs: MutableList<LynxModule?>

    override fun onInit() {
        allHubs = hardwareMap.getAll<LynxModule?>(LynxModule::class.java)

        val intake = button { gamepad1.circle }.whenBecomesTrue { Tube.intakeAll.schedule() }
        Turret.setTargetAngle(0.0.deg)
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        // telemetry.msTransmissionInterval = 100
        val selectRed =
            button { gamepad1.circle }.whenBecomesTrue { BotState.alliance = Alliance.RED }
        val selectBlue =
            button { gamepad1.cross }.whenBecomesTrue { BotState.alliance = Alliance.BLUE }
    }

    private fun buildRoutine(paths: Map<String, PathChain>): Command {
        val middleTurretAngle =
            when (BotState.alliance) {
                Alliance.RED -> AutoConstants.Angles["middleTurretRed"]
                Alliance.BLUE -> AutoConstants.Angles["middleTurretBlue"]
                else -> 0.0.deg
            }
        val intake: (Command) -> Command = { path ->
            SequentialGroup(Tube.intakeAll, path, Tube.shootAll(), Delay(400.milliseconds))
        }
        val gateIntake =
            intake(
                SequentialGroup(
                    FollowPath(paths.getValue("shootMiddleGateIntake")),
                    Tube.waitForAll(1100.milliseconds),
                    FollowPath(paths.getValue("gateIntakeShootMiddle")),
                )
            )
        return SequentialGroup(
            Flywheel.setSpeed(1_500.0),
            InstantCommand { Hood.position = 0.55 },
            InstantCommand { Turret.setTargetAngle(middleTurretAngle) },
            FollowPath(paths.getValue("startNearToShootMiddle")),
            Tube.shootAll(),
            Delay(400.milliseconds),
            intake(FollowPath(paths.getValue("spike2Combined"))),
            gateIntake,
            gateIntake,
            intake(FollowPath(paths.getValue("spike3Combined"))),
            intake(FollowPath(paths.getValue("spike1Combined"))),
            gateIntake,
            Flywheel.stop(),
        )
    }

    override fun onWaitForStart() {

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

        PedroComponent.follower.pose = poses.getValue("startNear")
        routine.schedule()
    }

    override fun onUpdate() {
        BotState.pose = PedroComponent.follower.pose
        telemetry.addData("X", BotState.pose?.x)
        telemetry.addData("Y", BotState.pose?.y)
        telemetry.addData("Heading", BotState.pose?.heading?.rad?.inDeg)
        telemetry.update()
        for (hub in allHubs) {
            hub!!.clearBulkCache()
        }
    }

    override fun onStop() {
        BotState.enabled = false
        Flywheel.setSpeed(0.0).schedule()
    }
}
