package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import dev.nextftc.extensions.pedro.PedroComponent

object PoseUtils {

  fun mirrorPose(pose: Pose): Pose {
    return Pose(pose.x, 144.0 - pose.y, pose.heading)
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

