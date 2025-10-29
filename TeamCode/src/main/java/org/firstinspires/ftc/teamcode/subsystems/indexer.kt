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

  fun toPosition(position: Double) =
      LambdaCommand("indexerToPosition")
          .setStart {
            spindexerPID.goal = KineticState(position, 0.0)
            goalPosition = position
          }
          .setIsDone {
            spindexerPID.isWithinTolerance(KineticState(100.0, 100.0, Double.POSITIVE_INFINITY))
          }
          .requires(this)

  fun toSlot(slot: Int) =
      LambdaCommand("indexerToSlot")
          .setStart {
            val cycleLength = 2730.0 * 3
            val slotPosition = slot * 2730.0

            val numCycles =
                kotlin.math.ceil(kotlin.math.abs(goalPosition) / cycleLength).toInt() + 2
            val positions = mutableListOf<Double>()

            for (cycle in -numCycles..numCycles) {
              positions.add(slotPosition + cycle * cycleLength)
            }

            val closestPosition =
                positions.minByOrNull { kotlin.math.abs(it - goalPosition) } ?: slotPosition

            spindexerPID.goal = KineticState(closestPosition, 0.0)
            goalPosition = closestPosition
          }
          .setIsDone {
            spindexerPID.isWithinTolerance(KineticState(100.0, 100.0, Double.POSITIVE_INFINITY))
          }
          .requires(this)

  fun toNextSlot() = toPosition(goalPosition + 2730.0)

  fun toPreviousSlot() = toPosition(goalPosition - 2730.0)

  fun latchDown() = LambdaCommand("latchDown").setStart { latch.position = 1.0 }.setIsDone { true }

  fun latchUp() = LambdaCommand("latchUp").setStart { latch.position = 0.6 }.setIsDone { true }
}
