package org.firstinspires.ftc.teamcode.opmodes.autos

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
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

@Autonomous(name = "Near 21", group = "Near", preselectTeleOp = "teleop")
class near21 : NextFTCOpMode() {
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

    val targetPose: Pose? = null

    private lateinit var allHubs: MutableList<LynxModule?>

    override fun onInit() {
        allHubs = hardwareMap.getAll<LynxModule?>(LynxModule::class.java)

        val intake = button { gamepad1.circle }.whenBecomesTrue { Tube.intakeAll.schedule() }
        Turret.setTargetAngle(0.0.deg)
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        // telemetry.msTransmissionInterval = 100
        val selectRed =
            button { gamepad1.left_bumper }.whenBecomesTrue { BotState.alliance = Alliance.RED }
        val selectBlue =
            button { gamepad1.right_bumper }.whenBecomesTrue { BotState.alliance = Alliance.BLUE }
    }

    private fun buildRoutine(paths: Map<String, PathChain>): Command {
        val middleTurretAngle =
            when (BotState.alliance) {
                Alliance.RED -> AutoConstants.Angles["middleTurretRed"]
                Alliance.BLUE -> AutoConstants.Angles["middleTurretBlue"]
                else -> 0.0.deg
            }
        val parkTurretAngle =
            when (BotState.alliance) {
                Alliance.RED -> AutoConstants.Angles["parkTurretRed3"]
                Alliance.BLUE -> AutoConstants.Angles["parkTurretBlue3"]
                else -> 0.0.deg
            }
        val startTurretAngle =
            when (BotState.alliance) {
                Alliance.RED -> AutoConstants.Angles["startTurretRed"]
                Alliance.BLUE -> AutoConstants.Angles["startTurretBlue3"]
                else -> 0.0.deg
            }
        val intake: (Command) -> Command = { path ->
            SequentialGroup(Tube.intakeAll, path, Tube.shootAll(), Delay(500.milliseconds))
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
            /*Flywheel.setSpeed(1_700.0),
            InstantCommand { Hood.position = 0.65 },
            InstantCommand { Turret.setTargetAngle(startTurretAngle) },*/
            ParallelGroup(
                FollowPath(paths.getValue("startNearToSpike2")),
                SequentialGroup(
                    InstantCommand { PedroComponent.follower.setMaxPower(0.7) },
                    Delay(800.milliseconds),
                    Tube.shootAll(),
                    Delay(200.milliseconds),
                    InstantCommand { PedroComponent.follower.setMaxPower(1.0) },
                    Delay(100.milliseconds),
                    Tube.intakeAll,
                ),
            ),
            /*Flywheel.setSpeed(1_500.0),
            InstantCommand { Hood.position = 0.65 },
            InstantCommand { Turret.setTargetAngle(middleTurretAngle) },*/
            FollowPath(paths.getValue("spike2ToShootMiddle")),
            Tube.shootAll(),
            Delay(200.milliseconds),
            gateIntake,
            gateIntake,
            intake(FollowPath(paths.getValue("spike1Combined"))),
            gateIntake,
            Flywheel.setSpeed(1_400.0),
            InstantCommand { Hood.position = 0.45 },
            InstantCommand { Turret.setTargetAngle(parkTurretAngle) },
            intake(FollowPath(paths.getValue("spike3Combined"))),
            Delay(500.milliseconds),
            Flywheel.stop(),
        )
    }

    override fun onWaitForStart() {

        val allianceDisplay = HtmlTelemetryUtils.createAllianceBadge(BotState.alliance)

        telemetry.addLine(allianceDisplay)
        telemetry.addLine("RED: ← bumper")
        telemetry.addLine("BLUE: → bumper")

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
        val targetMetrics =
            if (targetPose != null) {
                calculateTargetMetrics(
                    targetPose,
                    0.0,
                    Vector(),
                )
            } else {
                calculateTargetMetrics(
                    PedroComponent.follower.pose,
                    PedroComponent.follower.angularVelocity,
                    PedroComponent.follower.velocity,
                )
            }
        val distanceToTarget = targetMetrics.distanceToTarget
        val relativeAngleToTarget = targetMetrics.relativeAngleToTarget
        val config = ShootingConfigInterpolator.getConfig(distanceToTarget, ShootingConfigInterpolator.ShootingZone.NEAR)
        if (Flywheel.targetSpeed != config.flywheelSpeed) {
            Flywheel.setSpeedSafe(config.flywheelSpeed)
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
