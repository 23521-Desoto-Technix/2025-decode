package org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.DigitalChannel
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.Constants

object Indexer : Subsystem {
  val indexerServo = CRServoEx("indexer")
  val indexerEncoder = MotorEx("backRight").zeroed()
  val latchServo = ServoEx("latch")

  val intakeMotor = MotorEx("intake").brakeMode().reversed()

  val leftFeederServo = ServoEx("leftFeeder")
  val rightFeederServo = ServoEx("rightFeeder")

  val indexerPID = controlSystem { posPid(Constants.INDEXER_PID_P, Constants.INDEXER_PID_I, Constants.INDEXER_PID_D) }
  lateinit var leftBreakBeam: DigitalChannel
  lateinit var rightBreakBeam: DigitalChannel
  lateinit var intakeBreakBeam: DigitalChannel
  var currentPosition = 0.0
  var indexerPower = 0.0
  var intakePower = 0.0
  var goalPosition = 0.0
  var latched = false

  override fun periodic() {
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
            indexerPID.isWithinTolerance(KineticState(Constants.INDEXER_PID_TOLERANCE_POSITION, Constants.INDEXER_PID_TOLERANCE_VELOCITY, Double.POSITIVE_INFINITY))
          }
          .requires(this)

  fun waitForSlotBreakbeam() =
      LambdaCommand("waitForSlotBreakbeam").setIsDone {
        (!leftBreakBeam.state || !rightBreakBeam.state)
      }

  fun indexerToSlot(slot: Int) =
      LambdaCommand("indexerToSlot")
          .setStart {
            val cycleLength = Constants.INDEXER_SLOT_TICKS * Constants.INDEXER_CYCLE_SLOTS
            val slotPosition = slot * Constants.INDEXER_SLOT_TICKS

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

  fun toNextSlot() = indexerToPosition(goalPosition + Constants.INDEXER_SLOT_TICKS)

  fun setIntakePower(power: Double) =
      LambdaCommand("setIntakePower")
          .setStart { intakePower = power }
          .setIsDone { true }
          .requires(this)

  fun toPreviousSlot() = indexerToPosition(goalPosition - Constants.INDEXER_SLOT_TICKS)

  fun latchDown() =
      LambdaCommand("latchDown")
          .setStart {
            latchServo.position = Constants.LATCH_SERVO_DOWN
            latched = false
          }
          .setIsDone { true }

  fun latchUp() =
      LambdaCommand("latchUp")
          .setStart {
            latchServo.position = Constants.LATCH_SERVO_UP
            latched = true
          }
          .setIsDone { true }

  fun feed() =
      LambdaCommand("feed")
          .setStart {
            leftFeederServo.position = Constants.LEFT_FEEDER_FEED
            rightFeederServo.position = Constants.RIGHT_FEEDER_FEED
          }
          .setIsDone { true }
          .requires(this)

  fun unFeed() =
      LambdaCommand("resetFeeder")
          .setStart {
            leftFeederServo.position = Constants.LEFT_FEEDER_UNFEED
            rightFeederServo.position = Constants.RIGHT_FEEDER_UNFEED
          }
          .setIsDone { true }
          .requires(this)
}
