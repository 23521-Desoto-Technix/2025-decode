package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx

object Indexer : Subsystem {
    val servo = CRServoEx("indexer")
    val encoder = MotorEx("backRight").zeroed()
    val latch = ServoEx("latch")
    val spindexerPID = controlSystem { posPid(0.0002, 0.0, 0.00002) }
    lateinit var leftBreakBeam: DigitalChannel
    lateinit var rightBreakBeam: DigitalChannel
    lateinit var intakeBreakBeam: DigitalChannel
    var currentPosition = 0.0
    var power = 0.0
    var goalPosition = 0.0

    override fun periodic() {
        power = spindexerPID.calculate(KineticState(-encoder.currentPosition, -encoder.velocity))
        currentPosition = encoder.currentPosition.toDouble()
        servo.power = power

    }
    fun toPosition(position: Double) = LambdaCommand("indexerToPosition")
        .setStart {
            spindexerPID.goal = KineticState(position, 0.0)
            goalPosition = position
        }
        .setIsDone {
            spindexerPID.isWithinTolerance(KineticState(50.0, 50.0))
        }
        .requires(this)

    fun toSlot(slot: Int) = toPosition((slot % 3) * 2730.0)
    fun toNextSlot() = toPosition((((encoder.currentPosition / 2730.0).toInt() + 1) % 3) * 2730.0)
    fun toPreviousSlot() = toPosition((((encoder.currentPosition / 2730.0).toInt() - 1 + 3) % 3) * 2730.0)
}