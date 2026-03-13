package org.firstinspires.ftc.teamcode.opmodes

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.bindings.range
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.NextFTCOpMode
import java.util.Locale
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.BotConstants
import org.firstinspires.ftc.teamcode.TelemetryImplUpstreamSubmission
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Flywheel
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Tube
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.BotState
import org.firstinspires.ftc.teamcode.utils.PoseUtils.mirrorPose

data class ShootingConfig(
    val minDistance: Double,
    val maxDistance: Double,
    val flywheelSpeed: Double,
    val hoodPosition: Double,
)

@TeleOp
class teleop : NextFTCOpMode() {
    init {
        addComponents(
            // BulkReadComponent,
            BindingsComponent,
            PedroComponent(Constants::createFollower),
            SubsystemComponent(Tube, Shooter, Flywheel, Hood, Turret),
        )
        telemetry = TelemetryImplUpstreamSubmission(this)
    }

    var rotatedForward = 0.0
    var rotatedStrafe = 0.0
    var rotatedTurn = 0.0

    var ignorePinpoint = false

    var headingLocked = false

    var autoRangingEnabled = true

    private var lastUpdateNs = 0L

    val t = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)

    val shootingConfigs =
        listOf(
            ShootingConfig(60.0, 75.0, 1_400.0, 0.45),
            ShootingConfig(75.0, 85.0, 1_500.0, 0.55),
            ShootingConfig(85.0, 93.0, 1_500.0, 0.55),
            ShootingConfig(93.0, 98.0, 1_600.0, 0.7),
            ShootingConfig(98.0, 104.0, 1_600.0, 0.65),
            ShootingConfig(104.0, 110.0, 1_700.0, 0.65),
//            ShootingConfig(
//                110.0,
//                120.0,
//                900.0,
//                0.6,
                 //),
            //far zone
//                 ShootingConfig(
//                 120.0,
//                 135.0,
//                 1_950.0,
//                 0.9,
//                  ),
//                 ShootingConfig(
//                 135.0,
//                 160.0,
//                 2_050.0,
//                  0.95,
//            ),
        )

    val headingPID = controlSystem { posPid(0.0085, 0.0, 0.0) }

    var lockTurret = false

    private lateinit var backRight: com.qualcomm.robotcore.hardware.DcMotor
    private lateinit var frontLeft: com.qualcomm.robotcore.hardware.DcMotor
    private lateinit var backLeft: com.qualcomm.robotcore.hardware.DcMotor
    private lateinit var frontRight: com.qualcomm.robotcore.hardware.DcMotor
    private lateinit var pto: Servo

    val redReference = Pose(110.0, 131.3, 90.0.deg.inRad)
    val blueReference = mirrorPose(redReference)

    private lateinit var allHubs: MutableList<LynxModule?>

    fun rotateJoystickInput(
        forward: Double,
        strafe: Double,
        angle: dev.nextftc.core.units.Angle,
    ): Pair<Double, Double> {
        val angleRadians = angle.value
        val rotatedForward = forward * cos(angleRadians) - strafe * sin(angleRadians)
        val rotatedStrafe = forward * sin(angleRadians) + strafe * cos(angleRadians)
        return Pair(rotatedForward, rotatedStrafe)
    }

    fun getShootingConfigForDistance(distance: Double): ShootingConfig? {
        return shootingConfigs.firstOrNull {
            distance >= it.minDistance && distance < it.maxDistance
        }
    }

    override fun onInit() {
        backRight = hardwareMap.dcMotor["backRight"]
        frontLeft = hardwareMap.dcMotor["frontLeft"]
        backLeft = hardwareMap.dcMotor["backLeft"]
        frontRight = hardwareMap.dcMotor["frontRight"]
        pto = hardwareMap.servo["pto"]
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

        t.addLine(allianceDisplay)
        t.addLine("Controller 2")
        t.addLine("RED: Circle ●")
        t.addLine("BLUE: Cross ✕")

        BindingManager.update()
        t.update()
    }

    override fun onStartButtonPressed() {
        BotState.enabled = true
        val red = Pose(127.6, 120.8, -143.8.deg.inRad)
        val startingPose =
            when (BotState.alliance) {
                Alliance.RED -> red
                Alliance.BLUE -> mirrorPose(red)
                Alliance.UNKNOWN -> Pose(72.0, 72.0, 90.0.deg.inRad)
            }
        if (BotState.pose != null) {
            PedroComponent.follower.pose = BotState.pose!!
            if (BotState.alliance == Alliance.BLUE) {
                PedroComponent.follower.pose ==
                    Pose(
                        PedroComponent.follower.pose.x,
                        PedroComponent.follower.pose.y - 5,
                        PedroComponent.follower.pose.heading,
                    )
            }
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
                .whenBecomesTrue { Tube.shootAll().schedule() }
        val shootAllSlow =
            button { gamepad1.square }.whenBecomesTrue { Tube.shootAll(0.7).schedule() }

        val flywheelLong =
            button { gamepad1.dpad_up || gamepad2.dpad_up }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Hood.position = 0.935
                        Flywheel.enable().then(Flywheel.setSpeed(2_050.0)).schedule()
                    }
                }
        val flywheelShort =
            button { gamepad1.dpad_down || gamepad2.dpad_down }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Hood.position = 0.45
                        Flywheel.enable().then(Flywheel.setSpeed(1_600.0)).schedule()
                    }
                }
        val flywheelTesting =
            button { gamepad1.dpad_left || gamepad2.dpad_left }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Flywheel.enable().then(Flywheel.setSpeed(500.0)).schedule()
                    }
                }
        val flywheelOff =
            button { gamepad1.dpad_right || gamepad2.dpad_right }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Flywheel.disable().schedule()
                    }
                }
        val flywheelSpeedUp =
            button { gamepad2.left_trigger > 0.5 }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Flywheel.enable().then(Flywheel.setSpeed(Flywheel.targetSpeed + 100.0)).schedule()
                    }
                }
        val flywheelSpeedDown =
            button { gamepad2.right_trigger > 0.5 }
                .whenBecomesTrue {
                    if (!autoRangingEnabled) {
                        Flywheel.enable().then(Flywheel.setSpeed(maxOf(0.0, Flywheel.targetSpeed - 100.0))).schedule()
                    }
                }
        val hoodUp =
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
                }
        val driveCancel =
            button { abs(gamepad2.left_stick_y) > 0.1 }
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
        val headingLock =
            button { gamepad1.right_bumper }
                .whenTrue { headingLocked = true }
                .whenFalse { headingLocked = false }
        /*val slow =
        button { gamepad1.left_bumper }.whenTrue { slowMode = true }
            .whenFalse { slowMode = false }*/
        val autoAimToggle =
            button { gamepad2.ps }.whenBecomesTrue { autoRangingEnabled = !autoRangingEnabled }
        val lockTurretToggle =
            button { gamepad2.triangle }.whenBecomesTrue { lockTurret = !lockTurret }
        val resetPose =
            button { gamepad1.ps }
                .whenBecomesTrue {
                    if (BotState.alliance == Alliance.RED) {
                        PedroComponent.follower.pose = redReference
                    } else if (BotState.alliance == Alliance.BLUE) {
                        PedroComponent.follower.pose = blueReference
                    }
                }
    }

    override fun onUpdate() {

        for (hub in allHubs) {
            hub!!.clearBulkCache()
        }

        val nowNs = System.nanoTime()
        val loopMs = if (lastUpdateNs == 0L) 0.0 else (nowNs - lastUpdateNs) / 1_000_000.0
        lastUpdateNs = nowNs

        val targetPose =
            if (PedroComponent.follower.pose.y < 48.0) {
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
        val currentX = PedroComponent.follower.pose.x
        val currentY = PedroComponent.follower.pose.y
        val deltaX = targetPose.x - currentX
        val deltaY = targetPose.y - currentY
        val distanceToTarget = hypot(deltaX, deltaY)
        val absoluteAngleToTarget = atan2(deltaY, deltaX).rad
        val relativeAngleToTarget =
            (PedroComponent.follower.pose.heading.rad - absoluteAngleToTarget + 180.deg).normalized

        val config = getShootingConfigForDistance(distanceToTarget)
        if (config != null && autoRangingEnabled) {
            if (Flywheel.targetSpeed != config.flywheelSpeed) {
                Flywheel.enable().then(Flywheel.setSpeed(config.flywheelSpeed)).schedule()
            }
            if (Hood.position != config.hoodPosition) {
                Hood.position = config.hoodPosition
            }
        }

        if (ignorePinpoint) {
            if (((System.currentTimeMillis() / 500) % 2).toInt() == 0) {
                t.addLine(
                    "<span style=\"background-color: yellow; color: black;\">&nbsp;&nbsp;!!&nbsp;&nbsp;IGNORING PINPOINT&nbsp;&nbsp;!!&nbsp;&nbsp;</span>"
                )
            } else {
                t.addLine(
                    "&nbsp;&nbsp;!!&nbsp;&nbsp;IGNORING PINPOINT&nbsp;&nbsp;!!&nbsp;&nbsp;"
                )
            }
        }

        t.addData("X", PedroComponent.follower.pose.x)
        t.addData("Y", PedroComponent.follower.pose.y)
        t.addData("Heading", PedroComponent.follower.pose.heading)
        t.addData("Distance to Target", distanceToTarget)
        t.addData("Loop Time (ms)", String.format(Locale.US, "%.1f", loopMs))
        val shootingModeDisplay =
            if (autoRangingEnabled) {
                "<span style=\"background-color: #00FF00; color: black;\">&nbsp;&nbsp;AUTO&nbsp;&nbsp;</span>"
            } else {
                "<span style=\"background-color: yellow; color: black;\">&nbsp;&nbsp;MANUAL&nbsp;&nbsp;</span>"
            }
        t.addData("Shooting Mode", shootingModeDisplay)
        t.addData("Angle to (144, 144)", relativeAngleToTarget.inDeg)
        t.addData("Flywheel Target Speed", Flywheel.targetSpeed)
        t.addData("Flywheel Actual Speed", Flywheel.speed)
        t.addData("Hood position", Hood.position)
        /* HTML telemetry reference
        t.addLine("<b>Bold text</b>")
        t.addLine("<i>Italic text</i>")
        t.addLine("<u>Underlined text</u>")
        t.addLine("<font color=\"#FF0000\">Red text</font>")
        t.addLine("<font color=\"#00FF00\">Green text</font>")
        t.addLine("<b><i>Bold and italic</i></b>")
        t.addLine(
            "<span style=\"background-color: yellow; color: black;\">Yellow background</span>"
        )
        t.addLine(
            "<span style=\"background-color: #FF0000; color: white;\">Red background</span>"
        )
        t.addLine(
            "<span style=\"background-color: #0000FF; color: white;\">Blue background</span>"
        )*/

        var tatag = 0

        if (BotState.alliance == Alliance.RED) {
            tatag = BotConstants.RED_ALLIANCE_APRILTAG_ID
        } else if (BotState.alliance == Alliance.BLUE) {
            tatag = BotConstants.BLUE_ALLIANCE_APRILTAG_ID
        }

        if (abs(gamepad2.left_stick_y) > 0.1) {
            if (BotState.enabled) {
                backRight.power = gamepad2.left_stick_y.toDouble()
                frontRight.power = -gamepad2.left_stick_y.toDouble() * 0.75
                backLeft.power = gamepad2.left_stick_y.toDouble()
                frontLeft.power = -gamepad2.left_stick_y.toDouble() * 0.75
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

        if (headingLocked) {
            if (BotState.alliance == Alliance.RED) {
                headingPID.goal = KineticState(135.0, 0.0)
            } else if (BotState.alliance == Alliance.BLUE) {
                headingPID.goal = KineticState(45.0, 0.0)
            }

            rotatedTurn =
                headingPID.calculate(
                    KineticState(
                        PedroComponent.follower.pose.heading.rad.inDeg,
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

        /*
        if (!ignorePinpoint) {
          Turret.setTargetAngle(-relativeAngleToTarget)
        } else if (targetTagBearing == null) {
          Turret.setTargetAngle(0.0.deg)
        } else {
          Turret.setTargetAngle(Turret.currentAngle - targetTagBearing.deg)
        }*/

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
