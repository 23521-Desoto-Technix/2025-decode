package org.firstinspires.ftc.teamcode.opmodes.autos

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.conditionals.IfElseCommand
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
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
import org.firstinspires.ftc.teamcode.utils.HtmlTelemetryUtils
import org.firstinspires.ftc.teamcode.utils.ShootingConfigInterpolator
import org.firstinspires.ftc.teamcode.utils.calculateTargetMetrics
import kotlin.time.Duration.Companion.milliseconds

@Autonomous(name = "Near 24", group = "Near", preselectTeleOp = "teleop")
class near24 : NextFTCOpMode() {
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

    var targetPose: Pose? = null

    var forceCurrent = true

    var withPartner = false

    var doPark = false

    var gateBonk = false

    var roughTarget = Pose(0.0, 0.0, 0.0)

    private lateinit var poses: Map<String, Pose>

    private lateinit var allHubs: MutableList<LynxModule?>

    override fun onInit() {
        allHubs = hardwareMap.getAll<LynxModule?>(LynxModule::class.java)

        Turret.setTargetAngle(0.0.deg)
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)

        val intake = button { gamepad1.circle }.whenBecomesTrue { Tube.intakeAll.schedule() }
        val selectRed =
            button { gamepad1.left_bumper }.whenBecomesTrue { BotState.alliance = Alliance.RED }
        val selectBlue =
            button { gamepad1.right_bumper }.whenBecomesTrue { BotState.alliance = Alliance.BLUE }
        val toggleThirdSpike =
            button { gamepad1.triangle }.whenBecomesTrue { withPartner = !withPartner }
        val togglePark = button { gamepad1.cross }.whenBecomesTrue { doPark = !doPark }
        val toggleGateBonk = button { gamepad1.square }.whenBecomesTrue { gateBonk = !gateBonk }
    }

    private fun buildRoutine(paths: Map<String, PathChain>, poses: Map<String, Pose>): Command {
        val intake: (Command) -> Command = { path ->
            SequentialGroup(Tube.intakeAll, path, Tube.shootAll(), Delay(200.milliseconds))
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
            InstantCommand { roughTarget = poses.getValue("shootMiddle") },
            ParallelGroup(
                IfElseCommand(
                    { gateBonk },
                    FollowPath(paths.getValue("startNearToSpike2Gate")),
                    FollowPath(paths.getValue("startNearToSpike2")),
                ),
                SequentialGroup(
                    InstantCommand { PedroComponent.follower.setMaxPower(0.6) },
                    Delay(900.milliseconds),
                    Tube.shootAll(),
                    Delay(100.milliseconds),
                    InstantCommand { PedroComponent.follower.setMaxPower(1.0) },
                    InstantCommand { forceCurrent = false },
                    Delay(100.milliseconds),
                    Tube.intakeAll,
                ),
            ),
            FollowPath(paths.getValue("spike2ToShootMiddle")),
            Tube.shootAll(),
            Delay(200.milliseconds),
            gateIntake,
            gateIntake,
            intake(FollowPath(paths.getValue("spike1Combined"))),
            gateIntake,
            gateIntake,
            IfElseCommand(
                { withPartner },
                gateIntake,
                SequentialGroup(
                    InstantCommand { roughTarget = poses.getValue("shootPark") },
                    intake(FollowPath(paths.getValue("sideSpike3Combined"))),
                ),
            ),
            Delay(200.milliseconds),
            IfElseCommand(
                { doPark && withPartner },
                FollowPath(paths.getValue("shootMiddleToParkNear")),
            ),
            Flywheel.stop(),
            Flywheel.disable(),
        )
    }

    override fun onWaitForStart() {

        val allianceDisplay = HtmlTelemetryUtils.createAllianceBadge(BotState.alliance)

        telemetry.addLine(allianceDisplay)
        telemetry.addLine("RED: ← bumper")
        telemetry.addLine("BLUE: → bumper")
        telemetry.addLine("Toggle third spike: ▲")
        telemetry.addLine("Toggle park: ✕")
        telemetry.addLine("Toggle gate bonk: ■")
        telemetry.addLine("Intake: ●")

        val partnerBadge =
            if (withPartner) {
                HtmlTelemetryUtils.createColoredBadge("YES", "#00FF00", "black")
            } else {
                HtmlTelemetryUtils.createColoredBadge("NO", "#FF0000", "white")
            }
        telemetry.addData("For partner", partnerBadge)

        val parkBadge =
            if (doPark) {
                HtmlTelemetryUtils.createColoredBadge("YES", "#00FF00", "black")
            } else {
                HtmlTelemetryUtils.createColoredBadge("NO", "#FF0000", "white")
            }
        if (withPartner) {
            telemetry.addData("Do park", parkBadge)
        }
        val gateBonkBadge =
            if (gateBonk) {
                HtmlTelemetryUtils.createColoredBadge("YES", "#00FF00", "black")
            } else {
                HtmlTelemetryUtils.createColoredBadge("NO", "#FF0000", "white")
            }
        telemetry.addData("Bonk Gate", gateBonkBadge)

        BindingManager.update()
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        poses = AutoConstants.Poses.forAlliance(BotState.alliance)
        val paths = AutoConstants.Paths.forAlliance(BotState.alliance)
        routine = buildRoutine(paths, poses)

        PedroComponent.follower.pose = poses.getValue("startNear")
        routine.schedule()
    }

    override fun onUpdate() {
        val targetMetrics =
            if (targetPose != null) {
                calculateTargetMetrics(
                    targetPose!!,
                    0.0,
                    Vector(),
                )
            } else if (
                roughTarget.roughlyEquals(PedroComponent.follower.pose, 10.0) || forceCurrent
            ) {
                calculateTargetMetrics(
                    PedroComponent.follower.pose,
                    PedroComponent.follower.angularVelocity,
                    PedroComponent.follower.velocity,
                )
            } else {
                calculateTargetMetrics(
                    Pose(roughTarget.x, roughTarget.y, PedroComponent.follower.pose.heading),
                    0.0,
                    Vector(),
                )
            }
        val distanceToTarget = targetMetrics.distanceToTarget
        val relativeAngleToTarget = targetMetrics.relativeAngleToTarget
        val config =
            ShootingConfigInterpolator.getConfig(
                distanceToTarget,
                ShootingConfigInterpolator.ShootingZone.NEAR,
            )
        if (Flywheel.targetSpeed != config.flywheelSpeed) {
            Flywheel.setSpeedSafe(config.flywheelSpeed + 50)
        }
        if (Hood.position != config.hoodPosition) {
            Hood.position = config.hoodPosition
        }
        Turret.setTargetAngle(-relativeAngleToTarget)
        BotState.pose = PedroComponent.follower.pose
        telemetry.addData("X", BotState.pose?.x)
        telemetry.addData("Y", BotState.pose?.y)
        telemetry.addData("Heading", BotState.pose?.heading?.rad?.inDeg)
        try {
            telemetry.addData("Current path", PedroComponent.follower.currentPath.toString())
        } catch (e: Exception) {
            telemetry.addData("Current path", "None")
        }
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
