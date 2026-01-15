package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.utils.BotState.thenIfEnabled

object Shooter : SubsystemGroup(Tube, Flywheel) {
  val shootAllLong = Flywheel.setSpeed(2_200.0).then(Flywheel.waitForSpeed()).thenIfEnabled(Tube.shootAll())
  val shootAllShort = Flywheel.setSpeed(2_100.0).then(Flywheel.waitForSpeed()).thenIfEnabled(Tube.shootAll())
  val shootAllTesting = Flywheel.setSpeed(1_000.0).then(Flywheel.waitForSpeed()).thenIfEnabled(Tube.shootAll())
}
