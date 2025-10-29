package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.Feeder
import org.firstinspires.ftc.teamcode.subsystems.Indexer
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Lights
import org.firstinspires.ftc.teamcode.subsystems.LightsState
import org.firstinspires.ftc.teamcode.subsystems.Shooter
import org.firstinspires.ftc.teamcode.subsystems.Turret

@TeleOp
class ShooterTest : NextFTCOpMode() {
  init {
    addComponents(
        SubsystemComponent(Shooter, Intake, Indexer, Lights, Turret, Feeder),
        BulkReadComponent,
        BindingsComponent,
    )
  }

  private lateinit var intakeBreakBeam: DigitalChannel
  private lateinit var leftBreakBeam: DigitalChannel
  private lateinit var rightBreakBeam: DigitalChannel

  override fun onInit() {
    intakeBreakBeam = hardwareMap.get(DigitalChannel::class.java, "intakeBreakBeam")
    intakeBreakBeam.mode = DigitalChannel.Mode.INPUT
    leftBreakBeam = hardwareMap.get(DigitalChannel::class.java, "leftBreakBeam")
    leftBreakBeam.mode = DigitalChannel.Mode.INPUT
    rightBreakBeam = hardwareMap.get(DigitalChannel::class.java, "rightBreakBeam")
    rightBreakBeam.mode = DigitalChannel.Mode.INPUT
    SequentialGroup(
            InstantCommand { Lights.state = LightsState.DEBUG_GREEN },
            Indexer.toSlot(0),
            InstantCommand { Lights.state = LightsState.ALLIANCE_UNKNOWN },
        )
        .schedule()
  }

  override fun onWaitForStart() {}

  override fun onStartButtonPressed() {
    val bumpSpeedUp =
        button { gamepad1.right_bumper }
            .whenBecomesTrue { Shooter.setSpeed(Shooter.targetSpeed + 100.0).schedule() }
    val bumpSpeedDown =
        button { gamepad1.left_bumper }
            .whenBecomesTrue {
              Shooter.setSpeed((Shooter.targetSpeed - 100.0).coerceAtLeast(0.0)).schedule()
            }
    val intakeForward =
        button { gamepad1.circle }
            .toggleOnBecomesTrue()
            .whenBecomesTrue { Intake.setPower(1.0).schedule() }
            .whenBecomesFalse { Intake.setPower(0.0).schedule() }
    val turretStick =
        button { gamepad1.a } whenTrue
            {
              Turret.setPower(gamepad1.left_stick_x.toDouble()).schedule()
            } whenFalse
            {
              Turret.setPower(0.0).schedule()
            }
    val spindexerBumpNext =
        button { gamepad1.dpad_left } whenBecomesTrue
            {
              SequentialGroup(
                      InstantCommand { Lights.state = LightsState.DEBUG_PURPLE },
                      Indexer.toNextSlot(),
                      InstantCommand { Lights.state = LightsState.ALLIANCE_UNKNOWN },
                  )
                  .schedule()
            }
    val spindexerBumpPrevious =
        button { gamepad1.dpad_right } whenBecomesTrue
            {
              SequentialGroup(
                      InstantCommand { Lights.state = LightsState.DEBUG_PURPLE },
                      Indexer.toPreviousSlot(),
                      InstantCommand { Lights.state = LightsState.ALLIANCE_UNKNOWN },
                  )
                  .schedule()
            }
    val spindexerReset =
        button { gamepad1.dpad_down } whenBecomesTrue
            {
              SequentialGroup(
                      InstantCommand { Lights.state = LightsState.DEBUG_PURPLE },
                      Indexer.toSlot(0),
                      InstantCommand { Lights.state = LightsState.ALLIANCE_UNKNOWN },
                  )
                  .schedule()
            }
    val feed =
        button { gamepad1.y } whenTrue
            {
              Feeder.feed().schedule()
            } whenFalse
            {
              Feeder.reset().schedule()
            }
      val latch = button { gamepad1.square }
        .toggleOnBecomesTrue()
        .whenBecomesTrue { Indexer.latchDown().schedule() }
        .whenBecomesFalse { Indexer.latchUp().schedule() }
  }

  override fun onUpdate() {

    telemetry.addData("Shooter actual", Shooter.speed)
    telemetry.addData("Shooter target", Shooter.targetSpeed)
    telemetry.addData("Shooter power", Shooter.power)
    telemetry.addData("Turret angle", Turret.angle)
    telemetry.addData("Intake Break Beam", intakeBreakBeam.state)
    telemetry.addData("Left Break Beam", leftBreakBeam.state)
    telemetry.addData("Right Break Beam", rightBreakBeam.state)
    telemetry.addData("Indexer Position", Indexer.currentPosition)
    telemetry.addData("Indexer Goal", Indexer.goalPosition)
    telemetry.addData("Indexer Power", Indexer.power)
    telemetry.update()
    BindingManager.update()
  }

  override fun onStop() {
    BindingManager.reset()
  }
}
