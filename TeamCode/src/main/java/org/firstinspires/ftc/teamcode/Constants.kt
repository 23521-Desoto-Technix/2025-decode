package org.firstinspires.ftc.teamcode

/**
 * Constants used across the codebase.
 * All constants use SNAKE_CASE naming convention.
 */
object Constants {
  // Vision constants
  const val CAMERA_RESOLUTION_WIDTH = 800
  const val CAMERA_RESOLUTION_HEIGHT = 600
  const val CAMERA_LENS_FX = 667.154
  const val CAMERA_LENS_FY = 667.154
  const val CAMERA_LENS_CX = 438.702
  const val CAMERA_LENS_CY = 286.414

  // AprilTag IDs
  const val RED_ALLIANCE_APRILTAG_ID = 24
  const val BLUE_ALLIANCE_APRILTAG_ID = 20

  // Shooter speeds (RPM)
  const val SHOOTER_SPEED_FAR = 2_300.0
  const val SHOOTER_SPEED_CLOSE = 2_100.0
  const val SHOOTER_SPEED_OFF = 0.0

  // Intake power levels
  const val INTAKE_POWER_FORWARD = 1.0
  const val INTAKE_POWER_REVERSE = -1.0
  const val INTAKE_POWER_OFF = 0.0

  // Timing constants (milliseconds)
  const val WAIT_FOR_FEEDER_MS = 200
  const val WAIT_FOR_INDEXER_MS = 750
  const val LATCH_WAIT_SHORT_MS = 100
  const val LATCH_WAIT_LONG_MS = 500
  const val LOCK_BUFFER_MS = 150
  const val RUMBLE_DURATION_MS = 250

  // Gamepad thresholds
  const val TRIGGER_THRESHOLD = 0.5
  const val SPEED_MULTIPLIER_SLOW = 0.5
  const val SPEED_MULTIPLIER_NORMAL = 1.0

  // Hood adjustment
  const val HOOD_ADJUSTMENT_STEP = 0.1

  // Color detection HSV thresholds
  const val PURPLE_HUE_MIN = 200f
  const val PURPLE_HUE_MAX = 245f
  const val GREEN_HUE_MIN = 150f
  const val GREEN_HUE_MAX = 170f
  const val SATURATION_THRESHOLD = 140f

  // Indexer constants
  const val INDEXER_SLOT_TICKS = 2730.0
  const val INDEXER_CYCLE_SLOTS = 3
  const val INDEXER_SLOT_0 = 0
  const val INDEXER_SLOT_1 = 1
  const val INDEXER_SLOT_2 = 2

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

  // Field coordinates (inches)
  const val FIELD_CENTER_X = 72.0
  const val FIELD_CENTER_Y = 72.0
  const val FIELD_MAX_X = 144.0
  const val FIELD_MAX_Y = 144.0
  const val FIELD_MIN = 0.0

  // Path following speeds
  const val PATH_SPEED_SLOW = 0.25
  const val PATH_SPEED_MEDIUM = 0.3
  const val PATH_SPEED_FAST = 1.0

  // ERCompat autonomous path coordinates (inches)
  const val ER_RED_START_X = 80.1
  const val ER_RED_START_Y = 8.6
  const val ER_RED_SHOOT_X = 82.0
  const val ER_RED_SHOOT_Y = 10.0
  const val ER_RED_SPIKE_ONE_START_X = 97.0
  const val ER_RED_SPIKE_ONE_START_Y = 35.0
  const val ER_RED_SPIKE_ONE_END_X = 130.0
  const val ER_RED_SPIKE_ONE_END_Y = 35.0

  // Turret angles (degrees)
  const val TURRET_ANGLE_ER_START = 25.0
  const val TURRET_ANGLE_CLOSE_START = -45.0

  // Autonomous end wait time (seconds)
  const val AUTO_END_WAIT_SECONDS = 30

  // Close autonomous path coordinates (inches)
  const val CLOSE_RED_START_X = 111.1
  const val CLOSE_RED_START_Y = 133.3
  const val CLOSE_RED_SHOOT_X = 87.0
  const val CLOSE_RED_SHOOT_Y = 82.0
  const val CLOSE_RED_SPIKE_ONE_END_X = 125.0
  const val CLOSE_RED_SPIKE_ONE_END_Y = 85.0
  const val CLOSE_RED_SPIKE_TWO_START_X = 100.0
  const val CLOSE_RED_SPIKE_TWO_START_Y = 60.0
  const val CLOSE_RED_SPIKE_TWO_END_X = 130.0
  const val CLOSE_RED_SPIKE_TWO_END_Y = 60.0
  const val CLOSE_RED_GATE_HOVER_X = 115.0
  const val CLOSE_RED_GATE_HOVER_Y = 70.0

  // HSV conversion constants
  const val HSV_DELTA_THRESHOLD = 1e-6f
  const val HSV_HUE_SEGMENTS = 6f
  const val HSV_HUE_GREEN_OFFSET = 2f
  const val HSV_HUE_BLUE_OFFSET = 4f
  const val HSV_HUE_MULTIPLIER = 60f
  const val HSV_HUE_MAX = 360f
  const val HSV_SCALE_MAX = 255f
  const val HSV_CLAMP_MIN = 0f
  const val HSV_CLAMP_MAX = 1f
}
