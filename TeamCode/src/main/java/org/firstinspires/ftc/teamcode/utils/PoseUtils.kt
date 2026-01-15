package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent

object PoseUtils {

  fun mirrorPose(pose: Pose): Pose {
    return Pose(144.0 - pose.x, pose.y, (180.deg - pose.heading.rad).normalized.inRad)
  }

  fun createBasicPath(start: Pose, end: Pose): PathChain {
    return PedroComponent.follower
        .pathBuilder()
        .addPath(Path(BezierLine(start, end)))
        .setConstantHeadingInterpolation(0.0)
        .build()
  }


  fun createCurvedPath(start: Pose, controlPoint: Pose, end: Pose): PathChain {
    return PedroComponent.follower
        .pathBuilder()
        .addPath(Path(BezierCurve(start, controlPoint, end)))
        .setConstantHeadingInterpolation(0.0)
        .build()
  }
}
