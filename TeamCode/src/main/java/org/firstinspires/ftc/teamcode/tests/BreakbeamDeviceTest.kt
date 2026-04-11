package org.firstinspires.ftc.teamcode.tests

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.NextFTCOpMode

@TeleOp
class BreakbeamDeviceTest : NextFTCOpMode() {
    private lateinit var topDigitalDevice: com.qualcomm.robotcore.hardware.DigitalChannel
    private lateinit var middleDigitalDeviceA: com.qualcomm.robotcore.hardware.DigitalChannel
    private lateinit var middleDigitalDeviceB: com.qualcomm.robotcore.hardware.DigitalChannel
    private lateinit var bottomDigitalDevice: com.qualcomm.robotcore.hardware.DigitalChannel

    override fun onInit() {
        topDigitalDevice = hardwareMap.digitalChannel["top"]
        topDigitalDevice.mode = com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT

        middleDigitalDeviceA = hardwareMap.digitalChannel["middleA"]
        middleDigitalDeviceA.mode = com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT

        middleDigitalDeviceB = hardwareMap.digitalChannel["middleB"]
        middleDigitalDeviceB.mode = com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT

        bottomDigitalDevice = hardwareMap.digitalChannel["bottom"]
        bottomDigitalDevice.mode = com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT
    }

    override fun onUpdate() {
        telemetry.addData("Top Digital State", topDigitalDevice.state)
        telemetry.addData("Middle Alpha Digital State", middleDigitalDeviceA.state)
        telemetry.addData("Middle Beta Digital State", middleDigitalDeviceB.state)
        telemetry.addData("Bottom Digital State", bottomDigitalDevice.state)
        telemetry.update()
    }
}

