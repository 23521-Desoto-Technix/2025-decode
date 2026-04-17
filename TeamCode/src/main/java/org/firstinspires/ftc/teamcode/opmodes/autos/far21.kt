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
import kotlin.time.Duration.Companion.milliseconds
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.TelemetryImplUpstreamSubmission
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Tube
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.BotState
import org.firstinspires.ftc.teamcode.utils.HtmlTelemetryUtils

@Autonomous(name = "Far 21", group = "Far", preselectTeleOp = "teleop")
class far21 : NextFTCOpMode() {
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
        val farTurretAngle =
            when (BotState.alliance) {
                Alliance.RED -> AutoConstants.Angles["farTurretRed"]
                Alliance.BLUE -> AutoConstants.Angles["farTurretBlue"]
                else -> 0.0.deg
            }
        val parkTurretAngle =
            when (BotState.alliance) {
                Alliance.RED -> AutoConstants.Angles["parkTurretRed"]
                Alliance.BLUE -> AutoConstants.Angles["parkTurretBlue"]
                else -> 0.0.deg
            }
        val intake: (Command) -> Command = { path ->
            SequentialGroup(Tube.intakeAll, path, Tube.shootAll(), Delay(400.milliseconds))
        }
        return SequentialGroup(
            Flywheel.setSpeed(1_975.0),
            InstantCommand { Hood.position = 0.96 },
            InstantCommand { Turret.setTargetAngle(farTurretAngle) },
            FollowPath(paths.getValue("startFarToShootFar")),
            Flywheel.waitForSpeed(),
            Tube.shootAll(),
            Delay(400.milliseconds),
            intake(
                SequentialGroup(
                    FollowPath(paths.getValue("shootFarToHumanIntake")),
                    FollowPath(paths.getValue("humanIntakeToShootFar")),
                )
            ),
            intake(
                SequentialGroup(
                    FollowPath(paths.getValue("shootFarToSpike3")),
                    FollowPath(paths.getValue("spike3ToShootFar")),
                )
            ),
            intake(
                SequentialGroup(
                    FollowPath(paths.getValue("shootFarToWallIntakeA")),
                    FollowPath(paths.getValue("wallIntakeAToShootFar")),
                )
            ),
            intake(
                SequentialGroup(
                    FollowPath(paths.getValue("shootFarToWallIntakeB")),
                    FollowPath(paths.getValue("wallIntakeBToShootFar")),
                )
            ),
            intake(
                SequentialGroup(
                    FollowPath(paths.getValue("shootFarToWallIntakeC")),
                    FollowPath(paths.getValue("wallIntakeCToShootFar")),
                )
            ),
            intake(
                SequentialGroup(
                    FollowPath(paths.getValue("shootFarToWallIntakeD")),
                    FollowPath(paths.getValue("wallIntakeDToShootFar")),
                )
            ),
            intake(
                SequentialGroup(
                    FollowPath(paths.getValue("shootFarToWallIntakeE")),
                    FollowPath(paths.getValue("wallIntakeEToShootFar")),
                )
            ),
            intake(
                SequentialGroup(
                    FollowPath(paths.getValue("shootFarToSpike3Wall")),
                    FollowPath(paths.getValue("spike3ToShootFar")),
                )
            ),
            Flywheel.stop(),
        )
    }

    override fun onWaitForStart() {

        val allianceDisplay = HtmlTelemetryUtils.createAllianceBadge(BotState.alliance)

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

        PedroComponent.follower.pose = poses.getValue("startFar")
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
