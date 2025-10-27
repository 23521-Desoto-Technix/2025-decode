package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
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
            BindingsComponent
        )
    }

    override fun onInit() {
        Lights.state = LightsState.OFF
    }
    override fun onWaitForStart() { }
    override fun onStartButtonPressed() {
        Lights.state = LightsState.IDLE
        val bumpSpeedUp = button { gamepad1.right_bumper }.whenBecomesTrue { Shooter.setSpeed(Shooter.targetSpeed + 100.0).schedule() }
        val bumpSpeedDown = button { gamepad1.left_bumper }.whenBecomesTrue { Shooter.setSpeed((Shooter.targetSpeed - 100.0).coerceAtLeast(0.0)).schedule() }
        val intake = button { gamepad1.circle } whenTrue { Intake.setPower(1.0).schedule() } whenFalse { Intake.setPower(0.0).schedule() }
       // val intakeReverse = button { gamepad1.square } whenTrue { Intake.setPower(-1.0).schedule() } whenFalse { Intake.setPower(0.0).schedule() }
        val spinnyLeft = button { gamepad1.dpad_left } whenTrue { Indexer.setPower(1.0).schedule() } whenFalse { Indexer.setPower(0.0).schedule() }
        val spinnyRight = button { gamepad1.dpad_right } whenTrue { Indexer.setPower(-1.0).schedule() } whenFalse { Indexer.setPower(0.0).schedule() }
        val turretStick = button { gamepad1.a } whenTrue {
            Turret.setPower(gamepad1.left_stick_x.toDouble()).schedule()
        } whenFalse {
            Turret.setPower(0.0).schedule()
        }
        val feed = button { gamepad1.y } whenTrue { Feeder.feed().schedule() } whenFalse { Feeder.reset().schedule() }
    }
    override fun onUpdate() {
        telemetry.addData("Shooter actual", Shooter.speed)
        telemetry.addData("Shooter target", Shooter.targetSpeed)
        telemetry.addData("Shooter power", Shooter.power)
        telemetry.addData("Turret angle", Turret.angle)
        telemetry.update()
        BindingManager.update()
    }
    override fun onStop() {
        BindingManager.reset()
    }
}