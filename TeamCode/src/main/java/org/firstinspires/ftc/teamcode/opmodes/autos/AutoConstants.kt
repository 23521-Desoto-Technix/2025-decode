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
                pose("startNear", Pose(113.27, 134.27, -90.0.deg.inRad))
                pose("shootNear", Pose(110.0, 98.0, -90.0.deg.inRad))
                pose("sideSpike1", Pose(115.5, 88.0, -90.0.deg.inRad))
                pose("sideSpike1Ctrl", Pose(118.0, 100.0, -90.0.deg.inRad))

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
        val red: Map<String, PathChain> by lazy {
            buildPathsFor(PedroComponent.follower, Poses.red)
        }
        val blue: Map<String, PathChain> by lazy {
            buildPathsFor(PedroComponent.follower, Poses.blue)
        }

        fun forAlliance(alliance: Alliance): Map<String, PathChain> {
            return if (alliance == Alliance.BLUE) blue else red
        }
    }

    object Angles {
        private val angles =
            linkedMapOf<String, Angle>().apply {
                angle("closeTurretRed", (-35.0).deg)
                angle("closeTurretBlue", 35.0.deg)
                angle("farTurretRed", (-113.0).deg)
                angle("farTurretBlue", 116.0.deg)
            }

        operator fun get(name: String): Angle = angles.getValue(name)

        private fun MutableMap<String, Angle>.angle(name: String, angle: Angle) {
            this[name] = angle
        }
    }

    private fun buildPathsFor(
        follower: Follower,
        poses: Map<String, Pose>,
    ): Map<String, PathChain> {
        fun p(name: String) = poses.getValue(name)

        return linkedMapOf<String, PathChain>().apply {
            path(
                "startNearToShootNear",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("startNear"), p("shootNear")))
                    .setConstantHeadingInterpolation(p("startNear").heading)
                    .build(),
            )
            path(
                "shootNearSideSpike1",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootNear"), p("sideSpike1Ctrl"), p("sideSpike1")))
                    .setConstantHeadingInterpolation(p("startNear").heading)
                    .addPath(BezierLine(p("sideSpike1"), p("shootNear")))
                    .setConstantHeadingInterpolation(p("startNear").heading)
                    .build(),
            )
        }
    }

    private fun MutableMap<String, PathChain>.path(name: String, path: PathChain) {
        this[name] = path
    }
}
