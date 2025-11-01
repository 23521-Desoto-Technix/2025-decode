package org.firstinspires.ftc.teamcode.opmodes

import android.util.Size
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import kotlin.time.Duration.Companion.seconds
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.Hood
import org.firstinspires.ftc.teamcode.subsystems.Indexer
import org.firstinspires.ftc.teamcode.subsystems.Lights
import org.firstinspires.ftc.teamcode.subsystems.LightsState
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@TeleOp
class teleop : NextFTCOpMode() {
  init {
    addComponents(
        SubsystemComponent(Shooter, Indexer, Lights, Turret, Hood),
        BulkReadComponent,
        BindingsComponent,
        PedroComponent(Constants::createFollower),
    )
  }

  enum class Alliance {
    RED,
    BLUE,
    UNKNOWN,
  }

  private lateinit var intakeBreakBeam: DigitalChannel
  private lateinit var leftBreakBeam: DigitalChannel
  private lateinit var rightBreakBeam: DigitalChannel

  val RESOLUTION_WIDTH: Int = 800
  val RESOLUTION_HEIGHT: Int = 600

  lateinit var aprilTag: AprilTagProcessor

  lateinit var portal: VisionPortal

  private var lastDetectedCenterX: Double = 0.0
  private var lastDetectionTime: Long = 0

  var speedMultiplier = 1.0

  var alliance = Alliance.UNKNOWN

  override fun onInit() {
    intakeBreakBeam = hardwareMap.get(DigitalChannel::class.java, "intakeBreakBeam")
    intakeBreakBeam.mode = DigitalChannel.Mode.INPUT
    leftBreakBeam = hardwareMap.get(DigitalChannel::class.java, "leftBreakBeam")
    leftBreakBeam.mode = DigitalChannel.Mode.INPUT
    rightBreakBeam = hardwareMap.get(DigitalChannel::class.java, "rightBreakBeam")
    rightBreakBeam.mode = DigitalChannel.Mode.INPUT
    Indexer.intakeBreakBeam = intakeBreakBeam
    Indexer.leftBreakBeam = leftBreakBeam
    Indexer.rightBreakBeam = rightBreakBeam
    Indexer.indexerToSlot(0).schedule()
    aprilTag =
        AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .setLensIntrinsics(667.154, 667.154, 438.702, 286.414)
            // ... these parameters are fx, fy, cx, cy.
            .build()

    portal =
        VisionPortal.Builder()
            .setCamera(hardwareMap.get<WebcamName?>(WebcamName::class.java, "turretCamera"))
            .setCameraResolution(Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(aprilTag)
            .build()
    PedroComponent.follower.pose = Pose(0.0, 0.0, 0.0)
  }

  override fun onWaitForStart() {
    when (alliance) {
      Alliance.RED -> Lights.state = LightsState.ALLIANCE_RED
      Alliance.BLUE -> Lights.state = LightsState.ALLIANCE_BLUE
      Alliance.UNKNOWN -> Lights.state = LightsState.ALLIANCE_UNKNOWN
    }
    if (gamepad1.circle) {
      alliance = Alliance.RED
    } else if (gamepad1.cross) {
      alliance = Alliance.BLUE
    }
  }

  override fun onStartButtonPressed() {
    val bumpSpeedUp =
        button { gamepad2.right_bumper }
            .whenBecomesTrue {
              Shooter.setPower((Shooter.power + 0.1).coerceAtMost(1.0)).schedule()
            }
    val bumpSpeedDown =
        button { gamepad2.left_bumper }
            .whenBecomesTrue {
              Shooter.setPower((Shooter.power - 0.1).coerceAtLeast(0.0)).schedule()
            }
    val nominalPower =
        button { gamepad2.right_trigger > 0.5 }
            .whenBecomesTrue { Shooter.setPower(0.90).schedule() }
    val noPower =
        button { gamepad2.left_trigger > 0.5 }.whenBecomesTrue { Shooter.setPower(0.0).schedule() }
    val intakeForward =
        button { gamepad2.circle }
            .whenBecomesTrue {
              Indexer.setIntakePower(if (Indexer.intakePower == 1.0) 0.0 else 1.0).schedule()
            }
    val intakeReverse =
        button { gamepad2.cross }
            .whenBecomesTrue {
              Indexer.setIntakePower(if (Indexer.intakePower == -1.0) 0.0 else -1.0).schedule()
            }
    val spindexerBumpNext =
        button { gamepad2.dpad_left } whenBecomesTrue { Indexer.toNextSlot().schedule() }
    val spindexerBumpPrevious =
        button { gamepad2.dpad_right } whenBecomesTrue { Indexer.toPreviousSlot().schedule() }
    val spindexerReset =
        button { gamepad2.dpad_down } whenBecomesTrue { Indexer.indexerToSlot(0).schedule() }

    val feed =
        button { gamepad2.triangle } whenTrue
            {
              Indexer.feed().schedule()
            } whenBecomesFalse
            {
              SequentialGroup(
                      Indexer.unFeed(),
                      Delay(0.2.seconds),
                      Indexer.toNextSlot(),
                  )
                  .schedule()
            }
    val latch =
        button { gamepad2.square }
            .toggleOnBecomesTrue()
            .whenBecomesTrue { Indexer.latchDown().schedule() }
            .whenBecomesFalse { Indexer.latchUp().schedule() }
    val hoodUp =
        button { gamepad1.dpad_up }
            .whenBecomesTrue {
              Hood.setPosition((Hood.position + 0.1).coerceAtMost(1.0)).schedule()
            }
    val hoodDown =
        button { gamepad1.dpad_down }
            .whenBecomesTrue {
              Hood.setPosition((Hood.position - 0.1).coerceAtLeast(0.0)).schedule()
            }
    val speedToggle =
        Gamepads.gamepad1.rightTrigger
            .atLeast(0.5)
            .whenBecomesTrue { InstantCommand { speedMultiplier = 0.5 }.schedule() }
            .whenBecomesFalse { InstantCommand { speedMultiplier = 1.0 }.schedule() }
    val driverControlled =
        PedroDriverControlled(
            Gamepads.gamepad1.leftStickY.map { -it * speedMultiplier },
            Gamepads.gamepad1.leftStickX.map { -it * speedMultiplier },
            Gamepads.gamepad1.rightStickX.map { -it * speedMultiplier },
            false,
        )
      driverControlled()
  }

  override fun onUpdate() {
    telemetry.addData("Alliance", alliance)
    telemetry.addData("X", PedroComponent.follower.pose.x)
    telemetry.addData("Y", PedroComponent.follower.pose.y)
    telemetry.addData("Heading", PedroComponent.follower.pose.heading)
    // telemetry.addData("Shooter actual", Shooter.speed)
    // telemetry.addData("Shooter target", Shooter.targetSpeed)
    // telemetry.addData("Shooter power", Shooter.power)
    telemetry.addData("Turret angle", Turret.angle)
    telemetry.addData("Turret power", Turret.power)
    telemetry.addData("Turret PD error", Turret.previousError)
    // telemetry.addData("Intake Break Beam", intakeBreakBeam.state)
    // telemetry.addData("Left Break Beam", leftBreakBeam.state)
    // telemetry.addData("Right Break Beam", rightBreakBeam.state)
    // telemetry.addData("Indexer Position", Indexer.currentPosition)
    // telemetry.addData("Indexer Goal", Indexer.goalPosition)
    // telemetry.addData("Indexer Power", Indexer.power)

    var pixelOffset = 0.0
    var rotationComp = gamepad1.right_stick_x * 150.0

    for (detection in aprilTag.detections) {
      // telemetry.addLine("-----April Tag Detection-----")
      // telemetry.addData("Tag ID", detection.id)
      // telemetry.addData("Tag Center X", detection.center.x)
      // telemetry.addData("Tag Center Y", detection.center.y)
      // BLUE: 20, RED: 24

      if (
          (detection.id == 24 && alliance == Alliance.RED) ||
              (detection.id == 20 && alliance == Alliance.BLUE)
      ) {
        lastDetectionTime = System.currentTimeMillis()
        lastDetectedCenterX = detection.center.x
        pixelOffset = detection.center.x - (RESOLUTION_WIDTH / 2.0)
        telemetry.addData("ATag Detected", true)
        telemetry.addData("ATag Center X", detection.center.x)
        telemetry.addData("ATag Offset from Center", pixelOffset)
      }
    }

    Turret.cameraTrackPower(pixelOffset + rotationComp).schedule()

    BindingManager.update()
    telemetry.update()
  }

  override fun onStop() {
    BindingManager.reset()
  }
}
