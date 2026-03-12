package org.firstinspires.ftc.teamcode

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.utils.Motif

object BotConstants {
  const val CAMERA_RESOLUTION_WIDTH = 800
  const val CAMERA_RESOLUTION_HEIGHT = 600
  const val CAMERA_LENS_FX = 667.154
  const val CAMERA_LENS_FY = 667.154
  const val CAMERA_LENS_CX = 438.702
  const val CAMERA_LENS_CY = 286.414

  const val RED_ALLIANCE_APRILTAG_ID = 24
  const val BLUE_ALLIANCE_APRILTAG_ID = 20

  const val MOTIF_ATAG_ID_GPP = 21
  const val MOTIF_ATAG_ID_PGP = 22
  const val MOTIF_ATAG_ID_PPG = 23

  fun motifForAprilTagId(id: Int): Motif =
      when (id) {
        MOTIF_ATAG_ID_GPP -> Motif.GPP
        MOTIF_ATAG_ID_PGP -> Motif.PGP
        MOTIF_ATAG_ID_PPG -> Motif.PPG
        else -> Motif.UNKNOWN
      }

  val FIELD_CENTER = Pose(72.0, 72.0, 0.0)
  val RED_TARGET_POSE = Pose(144.0, 144.0, 0.0)
  val BLUE_TARGET_POSE = Pose(0.0, 144.0, 0.0)
}