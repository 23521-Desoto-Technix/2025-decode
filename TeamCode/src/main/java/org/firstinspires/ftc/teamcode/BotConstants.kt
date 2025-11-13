package org.firstinspires.ftc.teamcode

import com.pedropathing.geometry.Pose

/**
 * Shared constants used across multiple opmodes.
 * All constants use SNAKE_CASE naming convention.
 */
object BotConstants {
  // Vision constants - shared across teleop and autonomous opmodes
  const val CAMERA_RESOLUTION_WIDTH = 800
  const val CAMERA_RESOLUTION_HEIGHT = 600
  const val CAMERA_LENS_FX = 667.154
  const val CAMERA_LENS_FY = 667.154
  const val CAMERA_LENS_CX = 438.702
  const val CAMERA_LENS_CY = 286.414

  // AprilTag IDs - shared across teleop and autonomous opmodes
  const val RED_ALLIANCE_APRILTAG_ID = 24
  const val BLUE_ALLIANCE_APRILTAG_ID = 20

  // Shooter speeds (RPM) - shared across teleop and autonomous opmodes
  const val SHOOTER_SPEED_FAR = 2_300.0
  const val SHOOTER_SPEED_CLOSE = 2_100.0
  const val SHOOTER_SPEED_OFF = 0.0

  // Intake power levels - shared across teleop and autonomous opmodes
  const val INTAKE_POWER_FORWARD = 1.0
  const val INTAKE_POWER_REVERSE = -1.0
  const val INTAKE_POWER_OFF = 0.0

  // Timing constants (milliseconds) - shared across autonomous opmodes
  const val WAIT_FOR_FEEDER_MS = 200
  const val WAIT_FOR_INDEXER_MS = 750
  const val LATCH_WAIT_SHORT_MS = 100
  const val LATCH_WAIT_LONG_MS = 500

  // Indexer constants - shared across subsystems and opmodes
  const val INDEXER_SLOT_TICKS = 2730.0
  const val INDEXER_CYCLE_SLOTS = 3
  const val INDEXER_SLOT_0 = 0
  const val INDEXER_SLOT_1 = 1
  const val INDEXER_SLOT_2 = 2

  // Indexer servo positions - used by indexer subsystem
  const val LATCH_SERVO_DOWN = 0.90
  const val LATCH_SERVO_UP = 0.45
  const val LATCH_SERVO_STANDBY = 0.95
  const val LEFT_FEEDER_FEED = 0.61
  const val RIGHT_FEEDER_FEED = 0.71
  const val LEFT_FEEDER_UNFEED = 0.9
  const val RIGHT_FEEDER_UNFEED = 1.0

  // Indexer PID constants - used by indexer subsystem
  const val INDEXER_PID_P = 0.00015
  const val INDEXER_PID_I = 0.0
  const val INDEXER_PID_D = 0.000001

  // Indexer PID tolerance - used by indexer subsystem
  const val INDEXER_PID_TOLERANCE_POSITION = 100.0
  const val INDEXER_PID_TOLERANCE_VELOCITY = 100.0

  // Shared field poses - used in multiple opmodes
  val FIELD_CENTER = Pose(72.0, 72.0, 0.0)
  val RED_TARGET_POSE = Pose(144.0, 144.0, 0.0)
  val BLUE_TARGET_POSE = Pose(144.0, 0.0, 0.0)
}
