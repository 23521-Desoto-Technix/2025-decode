package org.firstinspires.ftc.teamcode.opmodes

import android.util.Size
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.bindings.range
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
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
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.atan2
import kotlin.time.Duration.Companion.seconds

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

  lateinit var aprilTag: AprilTagProcessor
  lateinit var portal: VisionPortal

  val RESOLUTION_WIDTH: Int = 800
  val RESOLUTION_HEIGHT: Int = 600

  var targetPose = Pose(72.0, 144.0, 0.0)
  var targetAprilTag = 0

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
    PedroComponent.follower.pose = Pose(134.7, 81.1, 0.0)
    PedroComponent.follower.breakFollowing()

    aprilTag =
        AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .setLensIntrinsics(667.154, 667.154, 438.702, 286.414)
            .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
            // ... these parameters are fx, fy, cx, cy.
            .build()
    aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN)

    portal =
        VisionPortal.Builder()
            .setCamera(hardwareMap.get<WebcamName?>(WebcamName::class.java, "turretCamera"))
            .setCameraResolution(Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(aprilTag)
            .build()
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
    val shooterFar =
        button { gamepad2.right_bumper }.whenBecomesTrue { Shooter.setSpeed(2_200.0).schedule() }
    val shooterClose =
        button { gamepad2.left_bumper }.whenBecomesTrue { Shooter.setSpeed(2_100.0).schedule() }
    val shooterOff =
        button { gamepad2.left_trigger > 0.5 }.whenBecomesTrue { Shooter.setSpeed(0.0).schedule() }
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
    /*
    val turretRight =
        button { gamepad1.dpad_right }
            .whenBecomesTrue { Turret.setAngle((Turret.targetAngle + 45.0).deg, true).schedule() }
    val turretLeft =
        button { gamepad1.dpad_left }
            .whenBecomesTrue { Turret.setAngle((Turret.targetAngle - 45.0).deg, true).schedule() }
    val turretCenter =
        button { gamepad1.circle }.whenBecomesTrue { Turret.setAngle(0.0.deg, true).schedule() }
        */
    val speedToggle =
        Gamepads.gamepad1.rightTrigger
            .atLeast(0.5)
            .whenBecomesTrue { InstantCommand { speedMultiplier = 0.5 }.schedule() }
            .whenBecomesFalse { InstantCommand { speedMultiplier = 1.0 }.schedule() }
    val driverControlled =
        PedroDriverControlled(
            range { -gamepad1.left_stick_y.toDouble() * speedMultiplier },
            range { -gamepad1.left_stick_x.toDouble() * speedMultiplier },
            range { -gamepad1.right_stick_x.toDouble() * speedMultiplier },
            false,
        )
    driverControlled()
  }

  override fun onUpdate() {
    telemetry.addData("X", PedroComponent.follower.pose.x)
    telemetry.addData("Y", PedroComponent.follower.pose.y)
    telemetry.addData("Heading", PedroComponent.follower.pose.heading)
    telemetry.addData("Shooter Actual", Shooter.speed)
    telemetry.addData("Shooter Target", Shooter.targetSpeed)
    Turret.IMUDegrees = PedroComponent.follower.pose.heading.rad.inDeg

    var pixelOffset = 0.0
    var rotationComp = gamepad1.right_stick_x * 150.0
    var hasLock = false

    if (alliance == Alliance.RED) {
      targetAprilTag = 24
    }
    if (alliance == Alliance.BLUE) {
      targetAprilTag = 20
    }
    if (aprilTag.detections.isNotEmpty()) {
      for (detection in aprilTag.detections) {
        if (detection.id == targetAprilTag) {
          hasLock = true
          pixelOffset = detection.center.x - (RESOLUTION_WIDTH / 2.0)
          if (detection.ftcPose != null) {
            telemetry.addData("pose", detection.ftcPose.range)
          } else {
            telemetry.addData("pose", "null")
          }
        }
      }
    }

    if (alliance == Alliance.RED) {
      targetPose = Pose(144.0, 144.0, 0.0)
    } else if (alliance == Alliance.BLUE) {
      targetPose = Pose(144.0, 0.0, 0.0)
    }
    val offsetX = targetPose.x + PedroComponent.follower.pose.x - 144.0
    val offsetY = targetPose.y - PedroComponent.follower.pose.y
    val goalAngle =
        atan2(
                offsetY,
                offsetX,
            )
            .rad
    telemetry.addData("Has Lock", hasLock)
    if (hasLock) {
      Turret.cameraTrackPower(pixelOffset).schedule()
    } else {
      Turret.setAngle(
              goalAngle,
              true,
          )
          .schedule()
    }

    BindingManager.update()
    telemetry.update()
  }

  override fun onStop() {
    BindingManager.reset()
  }
}
