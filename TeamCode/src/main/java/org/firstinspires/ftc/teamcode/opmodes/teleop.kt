package org.firstinspires.ftc.teamcode.opmodes

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.bindings.range
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.TelemetryImplUpstreamSubmission
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Tilt
import org.firstinspires.ftc.teamcode.subsystems.Tube
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.BotState
import org.firstinspires.ftc.teamcode.utils.HtmlTelemetryUtils
import org.firstinspires.ftc.teamcode.utils.PoseUtils.mirrorPose
import org.firstinspires.ftc.teamcode.utils.ShootingConfigInterpolator
import org.firstinspires.ftc.teamcode.utils.ShootingConfigInterpolator.ShootingZone
import java.util.Locale
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

data class TargetMetrics(val distanceToTarget: Double, val relativeAngleToTarget: Angle)

@TeleOp
class teleop : NextFTCOpMode() {
    init {
        addComponents(
            // BulkReadComponent,
            BindingsComponent,
            PedroComponent(Constants::createFollower),
            SubsystemComponent(Tube, Shooter, Flywheel, Hood, Turret, Tilt),
        )
        telemetry = TelemetryImplUpstreamSubmission(this)
    }

    var rotatedForward = 0.0
    var rotatedStrafe = 0.0
    var rotatedTurn = 0.0

    var ignorePinpoint = false

    var headingLocked: Angle? = null

    var autoRangingEnabled = true

    private var lastUpdateNs = 0L

    val t = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

    var activeShootingZone = ShootingZone.NEAR

    val headingPID = controlSystem { posPid(0.0085, 0.0, 0.0) }

    var lockTurret = false

    val MAX_LIFT = 59_500

    private lateinit var backRight: DcMotor
    private lateinit var frontLeft: DcMotor
    private lateinit var backLeft: DcMotor
    private lateinit var frontRight: DcMotor
    private lateinit var pto: Servo

    private lateinit var liftEncoder: DcMotor

    val redReferenceNear = Pose(109.6, 131.84, 90.0.deg.inRad)
    val blueReferenceNear = mirrorPose(redReferenceNear)

    val redReferenceFar = Pose(11.52, 9.3, -180.0.deg.inRad)
    val blueReferenceFar = mirrorPose(redReferenceFar)

    val redBase = Pose(34.3, 30.3, -135.deg.inRad)
    val blueBase = mirrorPose(redBase)

    private lateinit var allHubs: MutableList<LynxModule?>

    fun rotateJoystickInput(forward: Double, strafe: Double, angle: Angle): Pair<Double, Double> {
        val angleRadians = angle.value
        val rotatedForward = forward * cos(angleRadians) - strafe * sin(angleRadians)
        val rotatedStrafe = forward * sin(angleRadians) + strafe * cos(angleRadians)
        return Pair(rotatedForward, rotatedStrafe)
    }

    fun applyRobotSpaceOffset(pose: Pose, localX: Double, localY: Double): Pose {
        val heading = pose.heading
        val fieldX = pose.x + localX * cos(heading) - localY * sin(heading)
        val fieldY = pose.y + localX * sin(heading) + localY * cos(heading)
        return Pose(fieldX, fieldY, pose.heading)
    }

    fun calculateTargetMetrics(
        currentPose: Pose,
        angularVelocity: Double = 0.0,
        velocity: Vector,
    ): TargetMetrics {
        val targetPose =
            if (currentPose.y < 48.0) {
                if (BotState.alliance == Alliance.BLUE) {
                    Pose(4.0, 140.0, 0.0)
                } else {
                    Pose(140.0, 140.0, 0.0)
                }
            } else {
                if (BotState.alliance == Alliance.BLUE) {
                    Pose(0.0, 144.0, 0.0)
                } else {
                    Pose(144.0, 144.0, 0.0)
                }
            }
        val velocity =
            velocity.times(
                90.milliseconds.inWholeMicroseconds.toDouble() /
                    1.seconds.inWholeMicroseconds.toDouble()
            )

        val currentX = currentPose.x + velocity.xComponent
        val currentY = currentPose.y + velocity.yComponent
        val deltaX = targetPose.x - currentX
        val deltaY = targetPose.y - currentY
        val distanceToTarget = hypot(deltaX, deltaY)

        val redAnglePoseA = Pose(144.0, 125.0, 0.0)
        val redAnglePoseB = Pose(125.0, 144.0, 0.0)
        val anglePoseA =
            if (BotState.alliance == Alliance.BLUE) mirrorPose(redAnglePoseA) else redAnglePoseA
        val anglePoseB =
            if (BotState.alliance == Alliance.BLUE) mirrorPose(redAnglePoseB) else redAnglePoseB
        val angleToPoseA = atan2(anglePoseA.y - currentY, anglePoseA.x - currentX)
        val angleToPoseB = atan2(anglePoseB.y - currentY, anglePoseB.x - currentX)
        val absoluteAngleToTarget =
            atan2(sin(angleToPoseA) + sin(angleToPoseB), cos(angleToPoseA) + cos(angleToPoseB)).rad
        val predictiveHeading =
            currentPose.heading.rad +
                (angularVelocity / 1.seconds.inWholeMicroseconds *
                        90.milliseconds.inWholeMicroseconds)
                    .rad
        val relativeAngleToTarget = (predictiveHeading - absoluteAngleToTarget + 180.deg).normalized

        return TargetMetrics(distanceToTarget, relativeAngleToTarget)
    }

    override fun onInit() {
        backRight = hardwareMap.dcMotor["backRight"]
        frontLeft = hardwareMap.dcMotor["frontLeft"]
        backLeft = hardwareMap.dcMotor["backLeft"]
        frontRight = hardwareMap.dcMotor["frontRight"]
        pto = hardwareMap.servo["pto"]
        liftEncoder = frontLeft
        t.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        // t.msTransmissionInterval = 100
        allHubs = hardwareMap.getAll<LynxModule?>(LynxModule::class.java)

        val selectRed =
            button { gamepad2.circle }
                .inLayer("init")
                .whenBecomesTrue { BotState.alliance = Alliance.RED }
        val selectBlue =
            button { gamepad2.cross }
                .inLayer("init")
                .whenBecomesTrue { BotState.alliance = Alliance.BLUE }
    }

    override fun onWaitForStart() {
        BindingManager.layer = "init"
        val allianceDisplay = HtmlTelemetryUtils.createAllianceBadge(BotState.alliance)

        t.addLine(allianceDisplay)
        t.addLine("Controller 2")
        t.addLine("RED: Circle ●")
        t.addLine("BLUE: Cross ✕")

        BindingManager.update()
        t.update()
    }

    override fun onStartButtonPressed() {
        BotState.enabled = true
        pto.position = 0.0
        Tube.jiggle()
        val red = Pose(127.6, 120.8, -143.8.deg.inRad)
        val startingPose =
            when (BotState.alliance) {
                Alliance.RED -> Pose(72.0, 72.0, 90.0.deg.inRad)
                Alliance.BLUE -> Pose(72.0, 72.0, 90.0.deg.inRad)
                Alliance.UNKNOWN -> Pose(72.0, 72.0, 90.0.deg.inRad)
            }
        if (BotState.pose != null) {
            PedroComponent.follower.pose = BotState.pose!!
        } else {
            PedroComponent.follower.pose = startingPose
        }

        val driverControlled =
            PedroDriverControlled(
                range { rotatedForward },
                range { rotatedStrafe },
                range { rotatedTurn },
            )
        driverControlled()
        BindingManager.layer = null

        val intake =
            button { gamepad1.circle || gamepad1.right_trigger > 0.2 }
                .whenBecomesTrue { Tube.intakeAll.schedule() }
        val stopIntake = button { gamepad1.cross }.whenBecomesTrue { Tube.stopAll.schedule() }
        val shootAll =
            button { gamepad1.triangle || gamepad1.left_trigger > 0.2 }
                .whenBecomesTrue {
                    if (activeShootingZone == ShootingZone.NEAR) {
                        Tube.shootAll().schedule()
                    } else {
                        Tube.shootAll(0.7).schedule()
                    }
                }
        val shootAllSlow =
            button { gamepad1.square }.whenBecomesTrue { Tube.shootAll(0.7).schedule() }

        val flywheelLong =
            button { gamepad2.dpad_up }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Hood.position = 0.935
                        Flywheel.enable().then(Flywheel.setSpeed(2_050.0)).schedule()
                    }
                }
        val flywheelShort =
            button { gamepad2.dpad_down }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Hood.position = 0.45
                        Flywheel.enable().then(Flywheel.setSpeed(1_600.0)).schedule()
                    }
                }
        val flywheelTesting =
            button { gamepad2.dpad_left }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Flywheel.enable().then(Flywheel.setSpeed(500.0)).schedule()
                    }
                }
        val flywheelOff =
            button { gamepad2.dpad_right }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Flywheel.disable().schedule()
                    }
                }
        val flywheelSpeedUp =
            button { gamepad2.left_trigger > 0.5 }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Flywheel.enable()
                            .then(Flywheel.setSpeed(Flywheel.targetSpeed + 100.0))
                            .schedule()
                    }
                }
        val flywheelSpeedDown =
            button { gamepad2.right_trigger > 0.5 }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Flywheel.enable()
                            .then(Flywheel.setSpeed(maxOf(0.0, Flywheel.targetSpeed - 100.0)))
                            .schedule()
                    }
                }
        /*val hoodUp =
            button { gamepad2.left_bumper }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Hood.bumpUp().schedule()
                    }
                }
        val hoodDown =
            button { gamepad2.right_bumper }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Hood.bumpDown().schedule()
                    }
                }*/

        val autoPark =
            button { gamepad2.left_bumper }
                .whenBecomesTrue {
                    FollowPath(
                        PedroComponent.follower
                            .pathBuilder()
                            .addPath(
                                BezierLine(
                                    PedroComponent.follower.pose,
                                    when (BotState.alliance) {
                                        Alliance.RED -> redBase
                                        else -> blueBase
                                    },
                                )
                            )
                            .setConstantHeadingInterpolation(
                                when (BotState.alliance) {
                                    Alliance.RED -> redBase.heading
                                    else -> blueBase.heading
                                }
                            )
                            .build(),
                        true,
                    )
                }
                .whenBecomesFalse {
                    PedroComponent.follower.breakFollowing()
                }

        val driveCancel =
            button { abs(gamepad2.left_stick_y) > 0.1 || gamepad2.left_bumper }
                .whenBecomesTrue { driverControlled.cancel() }
                .whenBecomesFalse { driverControlled.schedule() }
        val ptoOn =
            button { gamepad2.circle && gamepad2.ps }
                .whenBecomesTrue {
                    if (BotState.enabled) {
                        pto.position = 0.95
                    }
                }
        val ptoOff =
            button { gamepad2.cross }
                .whenBecomesTrue {
                    if (BotState.enabled) {
                        pto.position = 0.0
                    }
                }
        val ignorePinpointToggle =
            button { gamepad2.square }.whenBecomesTrue { ignorePinpoint = !ignorePinpoint }
        val baseHeading =
            button { gamepad1.right_bumper }
                .whenBecomesTrue {
                    headingLocked =
                        if (BotState.alliance == Alliance.RED) {
                            135.deg
                        } else {
                            45.deg
                        }
                }
                .whenBecomesFalse { headingLocked = null }
        val gateHeading =
            button { gamepad1.left_bumper }
                .whenBecomesTrue {
                    headingLocked =
                        if (BotState.alliance == Alliance.RED) {
                            33.deg
                        } else {
                            147.deg
                        }
                }
                .whenBecomesFalse { headingLocked = null }
        val autoAimToggle =
            button { gamepad2.ps }.whenBecomesTrue { autoRangingEnabled = !autoRangingEnabled }
        val lockTurretToggle =
            button { gamepad2.triangle }.whenBecomesTrue { lockTurret = !lockTurret }
        val shootingZoneNear =
            button { gamepad2.dpad_left }
                .whenBecomesTrue {
                    if (autoRangingEnabled) {
                        activeShootingZone = ShootingZone.NEAR
                    }
                }
        val shootingZoneFar =
            button { gamepad2.dpad_right }
                .whenBecomesTrue {
                    if (autoRangingEnabled) {
                        activeShootingZone = ShootingZone.FAR
                    }
                }
        val resetPose =
            button { gamepad1.ps }
                .whenBecomesTrue {
                    if (BotState.alliance == Alliance.RED) {
                        PedroComponent.follower.pose =
                            when (activeShootingZone) {
                                ShootingZone.NEAR -> redReferenceNear
                                ShootingZone.FAR -> redReferenceFar
                            }
                    } else if (BotState.alliance == Alliance.BLUE) {
                        PedroComponent.follower.pose =
                            when (activeShootingZone) {
                                ShootingZone.NEAR -> blueReferenceNear
                                ShootingZone.FAR -> blueReferenceFar
                            }
                    }
                }
        val tiltDown =
            button { gamepad1.dpad_down }
                .whenBecomesTrue {
                    Tilt.down().schedule()
                    pto.position = 0.95
                }
        val tiltUp = button { gamepad1.dpad_up }.whenBecomesTrue { Tilt.up().schedule() }
    }

    override fun onUpdate() {

        for (hub in allHubs) {
            hub!!.clearBulkCache()
        }

        val nowNs = System.nanoTime()
        val loopMs = if (lastUpdateNs == 0L) 0.0 else (nowNs - lastUpdateNs) / 1_000_000.0
        lastUpdateNs = nowNs

        val targetMetrics =
            calculateTargetMetrics(
                applyRobotSpaceOffset(PedroComponent.follower.pose, -1.633, 0.0),
                PedroComponent.follower.angularVelocity,
                PedroComponent.follower.velocity,
            )
        val distanceToTarget = targetMetrics.distanceToTarget
        val relativeAngleToTarget = targetMetrics.relativeAngleToTarget

        if (autoRangingEnabled) {
            val config = ShootingConfigInterpolator.getConfig(distanceToTarget, activeShootingZone)
            if (Flywheel.targetSpeed != config.flywheelSpeed) {
                Flywheel.enable().then(Flywheel.setSpeed(config.flywheelSpeed)).schedule()
            }
            if (Hood.position != config.hoodPosition) {
                Hood.position = config.hoodPosition
            }
        }

        if (ignorePinpoint) {
            t.addLine(
                HtmlTelemetryUtils.createFlashingBadge(
                    "!!  IGNORING PINPOINT  !!",
                    "yellow",
                    "black",
                )
            )
        }

        t.addData("X", PedroComponent.follower.pose.x)
        t.addData("Y", PedroComponent.follower.pose.y)
        t.addData("Heading", PedroComponent.follower.pose.heading)
        t.addData("Distance to Target", distanceToTarget)
        t.addData("Lift", liftEncoder.currentPosition)
        t.addData("Lift under $MAX_LIFT", liftEncoder.currentPosition < MAX_LIFT)
        t.addData("Loop Time (ms)", String.format(Locale.US, "%.1f", loopMs))
        val shootingModeDisplay =
            if (autoRangingEnabled) {
                HtmlTelemetryUtils.createColoredBadge("AUTO", "#00FF00", "black")
            } else {
                HtmlTelemetryUtils.createColoredBadge("MANUAL", "yellow", "black")
            }
        t.addData("Shooting Mode", shootingModeDisplay)
        val shootingZoneDisplay =
            when (activeShootingZone) {
                ShootingZone.NEAR ->
                    HtmlTelemetryUtils.createColoredBadge("NEAR", "#FFA500", "black")
                ShootingZone.FAR -> HtmlTelemetryUtils.createColoredBadge("FAR", "#9933FF", "white")
            }
        t.addData("Shooting Zone", shootingZoneDisplay)
        t.addData("Angle to (144, 144)", relativeAngleToTarget.inDeg)
        t.addData("Flywheel Target Speed", Flywheel.targetSpeed)
        t.addData("Flywheel Actual Speed", Flywheel.speed)
        t.addData("Hood position", Hood.position)

        if (abs(gamepad2.left_stick_y) > 0.1) {
            if (BotState.enabled && liftEncoder.currentPosition < MAX_LIFT) {
                backRight.power = gamepad2.left_stick_y.toDouble()
                backLeft.power = gamepad2.left_stick_y.toDouble()
            } else {
                backRight.power = 0.0
                frontRight.power = 0.0
                backLeft.power = 0.0
                frontLeft.power = 0.0
            }
        } else if (!BotState.enabled) {
            backRight.power = 0.0
            frontRight.power = 0.0
            backLeft.power = 0.0
            frontLeft.power = 0.0
        }
        BindingManager.update()
        t.update()
        var rotateBy = -PedroComponent.follower.pose.heading.rad
        if (BotState.alliance == Alliance.BLUE) {
            rotateBy = (rotateBy + 180.deg).normalized
        }
        if (ignorePinpoint) {
            rotateBy = 0.0.deg
        }
        val rotated =
            rotateJoystickInput(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                rotateBy,
            )
        rotatedForward = rotated.first
        rotatedStrafe = rotated.second

        /*if (slowMode) {
            rotatedForward *= 0.5
            rotatedStrafe *= 0.5
        }*/

        if (headingLocked != null) {
            headingPID.goal = KineticState(0.0, 0.0)

            rotatedTurn =
                headingPID.calculate(
                    KineticState(
                        -((headingLocked!! - PedroComponent.follower.pose.heading.rad)
                            .normalized
                            .inDeg),
                        PedroComponent.follower.angularVelocity.rad.inDeg,
                    )
                )
        } else {
            rotatedTurn = -gamepad1.right_stick_x.toDouble()
        }
        if (!BotState.enabled) {
            rotatedForward = 0.0
            rotatedStrafe = 0.0
            rotatedTurn = 0.0
            return
        }

        if (!ignorePinpoint && !lockTurret) {
            Turret.setTargetAngle(-relativeAngleToTarget)
        } else {
            Turret.setTargetAngle(0.0.deg)
        }
    }

    override fun onStop() {
        BindingManager.reset()
    }
}
