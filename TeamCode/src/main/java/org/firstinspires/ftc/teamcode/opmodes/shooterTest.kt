package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.bindings.BindingManager
import dev.nextftc.bindings.button
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.subsystems.shooter

@TeleOp
class ShooterTest : NextFTCOpMode() {
    init {
        addComponents(/* vararg components */)
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
        BindingManager.update()
    }
    override fun onStop() {
        BindingManager.update()
    }
}