package org.firstinspires.ftc.teamcode

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.utils.Motif

/**
 * Shared constants used across multiple opmodes. All constants use SNAKE_CASE naming convention.
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

  // Motif AprilTag IDs
  const val MOTIF_ATAG_ID_GPP = 21
  const val MOTIF_ATAG_ID_PGP = 22
  const val MOTIF_ATAG_ID_PPG = 23

  /**
   * Map an AprilTag id to a Motif value. Returns Motif.UNKNOWN if no mapping exists.
   */
  fun motifForAprilTagId(id: Int): Motif =
      when (id) {
        MOTIF_ATAG_ID_GPP -> Motif.GPP
        MOTIF_ATAG_ID_PGP -> Motif.PGP
        MOTIF_ATAG_ID_PPG -> Motif.PPG
        else -> Motif.UNKNOWN
      }

  // Shooter speeds (RPM) - shared across teleop and autonomous opmodes
  const val SHOOTER_SPEED_FAR = 2_300.0
  const val SHOOTER_SPEED_CLOSE = 2_100.0
  const val SHOOTER_SPEED_OFF = 0.0

  // Intake power levels - shared across teleop and autonomous opmodes
  const val INTAKE_POWER_FORWARD = 0.8
  const val INTAKE_POWER_REVERSE = -1.0
  const val INTAKE_POWER_OFF = 0.0

  // Timing constants (milliseconds) - shared across autonomous opmodes
  const val WAIT_FOR_FEEDER_MS = 200
  const val WAIT_FOR_INDEXER_MS = 750
  const val LATCH_WAIT_SHORT_MS = 100
  const val LATCH_WAIT_LONG_MS = 500

  // Shared field poses - used in multiple opmodes
  val FIELD_CENTER = Pose(72.0, 72.0, 0.0)
  val RED_TARGET_POSE = Pose(144.0, 144.0, 0.0)
  val BLUE_TARGET_POSE = Pose(0.0, 144.0, 0.0)
}