package org.firstinspires.ftc.teamcode.opmodes

import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx

class robotCentric : NextFTCOpMode() {
    init {
        addComponents(
             BulkReadComponent,
             BindingsComponent,
        )
    }
    private val frontLeftMotor = MotorEx("frontLeft").brakeMode().reversed()
    private val frontRightMotor = MotorEx("frontRight").brakeMode()
    private val backLeftMotor = MotorEx("backLeft").brakeMode().reversed()
    private val backRightMotor = MotorEx("backRight").brakeMode()

    val driverControlled = MecanumDriverControlled(
    frontLeftMotor,
    frontRightMotor,
    backLeftMotor,
    backRightMotor,
    Gamepads.gamepad1.leftStickY,
    Gamepads.gamepad1.leftStickX,
    Gamepads.gamepad1.rightStickX
    )

    override fun onInit() {}
    override fun onWaitForStart() {}
    override fun onStartButtonPressed() {
        driverControlled()
    }
    override fun onUpdate() {}
    override fun onStop() {}
}