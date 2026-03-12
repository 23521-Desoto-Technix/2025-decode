package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.geometry.Pose
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand

object BotState {
  var alliance: Alliance = Alliance.UNKNOWN
  var pose: Pose? = null
  var enabled = true

  fun Command.thenIfEnabled(next: Command): Command = this.then(
      InstantCommand {
          if (BotState.enabled) {
              next.schedule()
          }
      }
  )
}
