package org.firstinspires.ftc.teamcode.utils

import org.firstinspires.ftc.teamcode.BotConstants

enum class Motif {
  PPG,
  PGP,
  GPP,
  UNKNOWN;

  override fun toString(): String =
      when (this) {
        PPG -> "PPG"
        PGP -> "PGP"
        GPP -> "GPP"
        UNKNOWN -> "UNKNOWN"
      }

  val isKnown: Boolean
    get() = this != UNKNOWN

  companion object {
    /** Map an AprilTag id to a Motif using constants from BotConstants. */
    fun fromAprilTagId(id: Int): Motif =
        when (id) {
          BotConstants.MOTIF_ATAG_ID_GPP -> GPP
          BotConstants.MOTIF_ATAG_ID_PGP -> PGP
          BotConstants.MOTIF_ATAG_ID_PPG -> PPG
          else -> UNKNOWN
        }
  }
}