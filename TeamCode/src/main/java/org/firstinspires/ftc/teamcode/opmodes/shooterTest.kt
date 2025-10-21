package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.Shooter

@TeleOp
class ShooterTest : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(Shooter),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() { }
    override fun onWaitForStart() { }
    override fun onStartButtonPressed() {
        val spinUpToggle = button { gamepad1.circle }
            .whenBecomesTrue(Shooter.setSpeed(2_600.0))
        val stopToggle = button { gamepad1.cross }
            .whenBecomesTrue(Shooter.setSpeed(0.0))
        val halfSpeedToggle = button { gamepad1.square }
            .whenBecomesTrue(Shooter.setSpeed(1_300.0))
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