package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import kotlin.time.Duration.Companion.seconds

object Indexer : Subsystem {
  val indexerServo = CRServoEx("indexer")
  val indexerEncoder = MotorEx("backRight").zeroed()
  val latchServo = ServoEx("latch")

  val intakeMotor = MotorEx("intake").brakeMode().reversed()

  val leftFeederServo = ServoEx("leftFeeder")
  val rightFeederServo = ServoEx("rightFeeder")

  val indexerPID = controlSystem { posPid(0.0001, 0.0, 0.000003) }
  lateinit var leftBreakBeam: DigitalChannel
  lateinit var rightBreakBeam: DigitalChannel
  lateinit var intakeBreakBeam: DigitalChannel
  var currentPosition = 0.0
  var indexerPower = 0.0
  var intakePower = 0.0
  var goalPosition = 0.0

  override fun periodic() {
    if (
        intakePower > 0 &&
            (!leftBreakBeam.state || !rightBreakBeam.state) &&
            latchServo.position == 0.95
    ) {
      SequentialGroup(
              Delay(0.05.seconds),
              this.latchUp(),
              Delay(0.1.seconds),
              this.toNextSlot(),
              Delay(0.3.seconds),
              this.latchDown(),
          )
          .schedule()
    }
    indexerPower =
        indexerPID.calculate(
            KineticState(-indexerEncoder.currentPosition, -indexerEncoder.velocity)
        )
    currentPosition = indexerEncoder.currentPosition.toDouble()
    indexerServo.power = indexerPower
    intakeMotor.power = intakePower
  }

  fun indexerToPosition(position: Double) =
      LambdaCommand("indexerToPosition")
          .setStart {
            indexerPID.goal = KineticState(position, 0.0)
            goalPosition = position
          }
          .setIsDone {
            true
          }
          .requires(this)

  fun waitForPid() =
      LambdaCommand("waitForPid")
          .setIsDone {
            indexerPID.isWithinTolerance(KineticState(100.0, 100.0, Double.POSITIVE_INFINITY))
          }
          .requires(this)

  fun indexerToSlot(slot: Int) =
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

            indexerPID.goal = KineticState(closestPosition, 0.0)
            goalPosition = closestPosition
          }
          .setIsDone { true }
          .requires(this)

  fun toNextSlot() = indexerToPosition(goalPosition + 2730.0)

  fun setIntakePower(power: Double) =
      LambdaCommand("setIntakePower")
          .setStart { intakePower = power }
          .setIsDone { true }
          .requires(this)

  fun toPreviousSlot() = indexerToPosition(goalPosition - 2730.0)

  fun latchDown() =
      LambdaCommand("latchDown").setStart { latchServo.position = 0.95 }.setIsDone { true }

  fun latchUp() =
      LambdaCommand("latchUp").setStart { latchServo.position = 0.45 }.setIsDone { true }

  fun feed() =
      LambdaCommand("feed")
          .setStart {
            leftFeederServo.position = 0.61
            rightFeederServo.position = 0.71
          }
          .setIsDone { true }
          .requires(this)

  fun unFeed() =
      LambdaCommand("resetFeeder")
          .setStart {
            leftFeederServo.position = 0.9
            rightFeederServo.position = 1.0
          }
          .setIsDone { true }
          .requires(this)
}
