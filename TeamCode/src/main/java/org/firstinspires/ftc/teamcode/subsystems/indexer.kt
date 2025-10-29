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
    val spindexerPID = controlSystem { posPid(0.0001, 0.0, 0.000003) }
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
            spindexerPID.isWithinTolerance(KineticState(100.0, 100.0, Double.POSITIVE_INFINITY))
        }
        .requires(this)

    private fun getClosestSlotPosition(slot: Int): Double {
        val cycleLength = 2730.0 * 3
        val targetBase = (slot % 3) * 2730.0

        val current = currentPosition
        var target = targetBase

        while (target - current > cycleLength / 2) {
            target -= cycleLength
        }
        while (target - current < -cycleLength / 2) {
            target += cycleLength
        }

        return target
    }

    fun toSlot(slot: Int) = toPosition(getClosestSlotPosition(slot))
    fun toNextSlot() {
        val currentSlot = ((goalPosition / 2730.0).toInt() % 3 + 3) % 3
        val nextSlot = (currentSlot + 1) % 3
        toPosition(getClosestSlotPosition(nextSlot))
    }
    fun toPreviousSlot() {
        val currentSlot = ((goalPosition / 2730.0).toInt() % 3 + 3) % 3
        val previousSlot = (currentSlot - 1 + 3) % 3
        toPosition(getClosestSlotPosition(previousSlot))
    }
}