package org.firstinspires.ftc.teamcode.opmodes.autos

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.PoseUtils.mirrorPose

object AutoConstants {
  object Poses {
    private val redPoses =
        linkedMapOf<String, Pose>().apply {
          pose("start", Pose(127.6, 120.8, -143.8.deg.inRad))
          pose("shoot", Pose(85.0, 85.0, -45.0.deg.inRad))
          pose("spike1", Pose(125.0, 85.0, 0.0.deg.inRad))
          pose("spike2", Pose(123.0, 60.0, 0.0.deg.inRad))
          pose("gate", Pose(125.0, 70.0, 0.0.deg.inRad))
          pose("gateMid", Pose(110.0, 65.0, 0.0.deg.inRad))
          pose("gateBonk", Pose(130.0, 61.0, 25.deg.inRad))
          pose("gateIntake", Pose(128.0, 52.0, 32.deg.inRad))
          pose("spike3", Pose(125.0, 35.0, 0.0.deg.inRad))
          pose("park", Pose(110.0, 70.0, 0.0.deg.inRad))
          pose("fastPark", Pose(93.0, 77.0, -45.0.deg.inRad))
          pose("s2Ctrl", Pose(85.0, 55.0, 0.0))
          pose("s3Ctrl", Pose(85.0, 30.0, 0.0))
          pose("startFar", Pose(79.2, 7.8, 90.0.deg.inRad))
          pose("shootFar", Pose(85.0, 15.0, 0.0.deg.inRad))
        }

    val red: Map<String, Pose>
      get() = redPoses

    val blue: Map<String, Pose> by lazy { redPoses.mapValues { mirrorPose(it.value) } }

    fun forAlliance(alliance: Alliance): Map<String, Pose> {
      return if (alliance == Alliance.BLUE) blue else red
    }

    private fun MutableMap<String, Pose>.pose(name: String, pose: Pose) {
      this[name] = pose
    }
  }

  object Paths {
    val red: Map<String, PathChain> by lazy { buildPathsFor(PedroComponent.follower, Poses.red) }
    val blue: Map<String, PathChain> by lazy { buildPathsFor(PedroComponent.follower, Poses.blue) }

    fun forAlliance(alliance: Alliance): Map<String, PathChain> {
      return if (alliance == Alliance.BLUE) blue else red
    }
  }

  object Angles {
    private val angles =
        linkedMapOf<String, Angle>().apply {
          angle("closeTurretRed", (-90.0).deg)
          angle("closeTurretBlue", 90.0.deg)
          angle("farTurretRed", 115.0.deg)
          angle("farTurretBlue", 115.0.deg)
        }

    operator fun get(name: String): Angle = angles.getValue(name)

    private fun MutableMap<String, Angle>.angle(
        name: String,
        angle: Angle,
    ) {
      this[name] = angle
    }
  }

  private fun buildPathsFor(follower: Follower, poses: Map<String, Pose>): Map<String, PathChain> {
    fun p(name: String) = poses.getValue(name)

    return linkedMapOf<String, PathChain>().apply {
      path(
          "startToShoot",
          follower
              .pathBuilder()
              .addPath(BezierLine(p("start"), p("shoot")))
              .setLinearHeadingInterpolation(p("start").heading, p("shoot").heading)
              .build(),
      )
      path(
          "shootToSpike1",
          follower
              .pathBuilder()
              .addPath(BezierLine(p("shoot"), p("spike1")))
              .setConstantHeadingInterpolation(p("spike1").heading)
              .build(),
      )
      path(
          "spike1ToShoot",
          follower
              .pathBuilder()
              .addPath(BezierLine(p("spike1"), p("shoot")))
              .setLinearHeadingInterpolation(p("spike1").heading, p("shoot").heading)
              .build(),
      )
      path(
          "shootToSpike2",
          follower
              .pathBuilder()
              .addPath(BezierCurve(p("shoot"), p("s2Ctrl"), p("spike2")))
              .setConstantHeadingInterpolation(p("spike2").heading)
              .build(),
      )
      path(
          "spike2ToGate",
          follower
              .pathBuilder()
              .addPath(BezierCurve(p("spike2"), p("gateMid"), p("gate")))
              .setConstantHeadingInterpolation(p("spike2").heading)
              .build(),
      )
      path(
          "shootToGateIntake",
          follower
              .pathBuilder()
              .addPath(BezierCurve(p("shoot"), p("s2Ctrl"), p("gateBonk")))
              .setConstantHeadingInterpolation(p("gateBonk").heading)
              .build(),
      )
      path(
          "gateIntakeToShoot",
          follower
              .pathBuilder()
              .addPath(BezierLine(p("gateBonk"), p("shoot")))
              .setConstantHeadingInterpolation(p("shoot").heading)
              .build(),
      )
      path(
          "spike2ToShoot",
          follower
              .pathBuilder()
              .addPath(BezierLine(p("spike2"), p("shoot")))
              .setConstantHeadingInterpolation(p("shoot").heading)
              .build(),
      )
      path(
          "shootToSpike3",
          follower
              .pathBuilder()
              .addPath(BezierCurve(p("shoot"), p("s3Ctrl"), p("spike3")))
              .setConstantHeadingInterpolation(p("spike3").heading)
              .build(),
      )
      path(
          "spike3ToShoot",
          follower
              .pathBuilder()
              .addPath(BezierLine(p("spike3"), p("shoot")))
              .setConstantHeadingInterpolation(p("shoot").heading)
              .build(),
      )
      path(
          "shootToPark",
          follower
              .pathBuilder()
              .addPath(BezierLine(p("shoot"), p("park")))
              .setConstantHeadingInterpolation(p("park").heading)
              .build(),
      )
      path(
          "shootToFastPark",
          follower
              .pathBuilder()
              .addPath(BezierLine(p("shoot"), p("fastPark")))
              .setConstantHeadingInterpolation(p("fastPark").heading)
              .build(),
      )
      path(
          "farStartToShoot",
          follower
              .pathBuilder()
              .addPath(BezierLine(p("startFar"), p("shootFar")))
              .setConstantHeadingInterpolation(p("shootFar").heading)
              .build(),
      )
    }
  }

  private fun MutableMap<String, PathChain>.path(name: String, path: PathChain) {
    this[name] = path
  }
}
