package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.BotConstants

object Indexer : Subsystem {
  // Indexer constants
  const val INDEXER_SLOT_TICKS = 2730.0
  const val INDEXER_CYCLE_SLOTS = 3
  
  // Indexer servo positions
  const val LATCH_SERVO_DOWN = 0.90
  const val LATCH_SERVO_UP = 0.45
  const val LATCH_SERVO_STANDBY = 0.95
  const val LEFT_FEEDER_FEED = 0.61
  const val RIGHT_FEEDER_FEED = 0.71
  const val LEFT_FEEDER_UNFEED = 0.9
  const val RIGHT_FEEDER_UNFEED = 1.0
  
  // Indexer PID constants
  const val INDEXER_PID_P = 0.00015
  const val INDEXER_PID_I = 0.0
  const val INDEXER_PID_D = 0.000001
  
  // Indexer PID tolerance
  const val INDEXER_PID_TOLERANCE_POSITION = 100.0
  const val INDEXER_PID_TOLERANCE_VELOCITY = 100.0

  val indexerServo = CRServoEx("indexer")
  val indexerEncoder = MotorEx("backRight").zeroed()
  val latchServo = ServoEx("latch")

  val intakeMotor = MotorEx("intake").brakeMode().reversed()

  val leftFeederServo = ServoEx("leftFeeder")
  val rightFeederServo = ServoEx("rightFeeder")

  val indexerPID = controlSystem { posPid(INDEXER_PID_P, INDEXER_PID_I, INDEXER_PID_D) }
  lateinit var leftBreakBeam: DigitalChannel
  lateinit var rightBreakBeam: DigitalChannel
  lateinit var intakeBreakBeam: DigitalChannel
  var currentPosition = 0.0
  var indexerPower = 0.0
  var intakePower = 0.0
  var goalPosition = 0.0
  var latched = false
  var enabled = true

  override fun periodic() {
    if (!enabled) {
      indexerServo.power = 0.0
      intakeMotor.power = 0.0
      return
    }


    /*if (
        intakePower > 0 &&
            (!leftBreakBeam.state || !rightBreakBeam.state) &&
            latchServo.position == 0.95
    ) {
      SequentialGroup(
              Delay(0.05.seconds),
              this.latchUp(),
              Delay(0.1.seconds),
              this.toNextSlot(),
              Delay(0.15.seconds),
              this.latchDown(),
          )
          .schedule()
    }*/
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
          .setIsDone { true }
          .requires(this)

  fun waitForPid() =
      LambdaCommand("waitForPid")
          .setIsDone {
            indexerPID.isWithinTolerance(KineticState(INDEXER_PID_TOLERANCE_POSITION, INDEXER_PID_TOLERANCE_VELOCITY, Double.POSITIVE_INFINITY))
          }
          .requires(this)

  fun waitForSlotBreakbeam() =
      LambdaCommand("waitForSlotBreakbeam").setIsDone {
        (!leftBreakBeam.state || !rightBreakBeam.state)
      }

  fun indexerToSlot(slot: Int) =
      LambdaCommand("indexerToSlot")
          .setStart {
            val cycleLength = INDEXER_SLOT_TICKS * INDEXER_CYCLE_SLOTS
            val slotPosition = slot * INDEXER_SLOT_TICKS

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

  fun toNextSlot() = indexerToPosition(goalPosition + INDEXER_SLOT_TICKS)

  fun setIntakePower(power: Double) =
      LambdaCommand("setIntakePower")
          .setStart { intakePower = power }
          .setIsDone { true }
          .requires(this)

  fun toPreviousSlot() = indexerToPosition(goalPosition - INDEXER_SLOT_TICKS)

  fun latchDown() =
      LambdaCommand("latchDown")
          .setStart {
            latchServo.position = LATCH_SERVO_DOWN
            latched = false
          }
          .setIsDone { true }

  fun latchUp() =
      LambdaCommand("latchUp")
          .setStart {
            latchServo.position = LATCH_SERVO_UP
            latched = true
          }
          .setIsDone { true }

  fun feed() =
      LambdaCommand("feed")
          .setStart {
            leftFeederServo.position = LEFT_FEEDER_FEED
            rightFeederServo.position = RIGHT_FEEDER_FEED
          }
          .setIsDone { true }
          .requires(this)

  fun unFeed() =
      LambdaCommand("resetFeeder")
          .setStart {
            leftFeederServo.position = LEFT_FEEDER_UNFEED
            rightFeederServo.position = RIGHT_FEEDER_UNFEED
          }
          .setIsDone { true }
          .requires(this)

  fun enable() =
      LambdaCommand("enableIndexer")
          .setStart { enabled = true }
          .setIsDone { true }
          .requires(this)

  fun disable() =
      LambdaCommand("disableIndexer")
          .setStart { enabled = false }
          .setIsDone { true }
          .requires(this)
}
