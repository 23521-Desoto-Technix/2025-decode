package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.BotConstants

enum class Alliance {
  RED,
  BLUE,
  UNKNOWN;

  fun aprilTagId(): Int =
      when (this) {
        RED -> BotConstants.RED_ALLIANCE_APRILTAG_ID
        BLUE -> BotConstants.BLUE_ALLIANCE_APRILTAG_ID
        UNKNOWN -> 0
      }

  fun targetPose(): Pose =
      when (this) {
        RED -> BotConstants.RED_TARGET_POSE
        BLUE -> BotConstants.BLUE_TARGET_POSE
        UNKNOWN -> BotConstants.FIELD_CENTER
      }

  override fun toString(): String =
      when (this) {
        RED -> "RED"
        BLUE -> "BLUE"
        UNKNOWN -> "UNKNOWN"
      }

  val isKnown: Boolean
    get() = this != UNKNOWN
}
