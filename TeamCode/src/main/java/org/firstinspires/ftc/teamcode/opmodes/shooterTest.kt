package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.shooter

@TeleOp
class ShooterTest : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(shooter),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() { }
    override fun onWaitForStart() { }
    override fun onStartButtonPressed() {
        val spinUpToggle = button { gamepad1.a }
            .toggleOnBecomesTrue()
            .whenBecomesTrue(shooter.spinUp)
            .whenBecomesFalse(shooter.spinDown)
    }
    override fun onUpdate() {
        telemetry.addData("Shooter flywheel speed", shooter.speed)
        telemetry.update()
        BindingManager.update()
    }
    override fun onStop() {
        BindingManager.reset()
    }
}