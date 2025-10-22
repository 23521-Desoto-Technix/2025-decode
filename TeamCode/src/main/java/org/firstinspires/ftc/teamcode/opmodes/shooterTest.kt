package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Shooter

@TeleOp
class ShooterTest : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(Shooter, Intake),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() { }
    override fun onWaitForStart() { }
    override fun onStartButtonPressed() {
        val bumpSpeedUp = button { gamepad1.right_bumper }.whenBecomesTrue { Shooter.setSpeed(Shooter.targetSpeed + 100.0).schedule() }
        val bumpSpeedDown = button { gamepad1.left_bumper }.whenBecomesTrue { Shooter.setSpeed((Shooter.targetSpeed - 100.0).coerceAtLeast(0.0)).schedule() }
        val intake = button { gamepad1.a } whenTrue { Intake.setPower(1.0).schedule() } whenFalse { Intake.setPower(0.0).schedule() }
    }
    override fun onUpdate() {
        telemetry.addData("Shooter actual", Shooter.speed)
        telemetry.addData("Shooter target", Shooter.targetSpeed)
        telemetry.addData("Shooter power", Shooter.power)
        telemetry.update()
        BindingManager.update()
    }
    override fun onStop() {
        BindingManager.reset()
    }
}