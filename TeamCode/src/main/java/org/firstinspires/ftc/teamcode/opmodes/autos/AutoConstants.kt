package org.firstinspires.ftc.teamcode.opmodes.autos

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
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
                pose("shootMiddle", Pose(89.0, 80.0, 0.0.deg.inRad))
                pose("shootPark", Pose(85.0, 105.0, 0.0.deg.inRad))
                pose("sideSpike1", Pose(119.0, 88.0, -90.0.deg.inRad))
                pose("sideSpike1Ctrl", Pose(120.0, 100.0, -90.0.deg.inRad))
                pose("sideSpike2", Pose(121.0, 74.0, -90.0.deg.inRad))
                pose("sideSpike2Ctrl", Pose(121.0, 93.0, -90.0.deg.inRad))
                pose("gateIntake", Pose(130.5, 59.5, 40.0.deg.inRad))
                pose("gateIntakeCtrl", Pose(105.0, 58.0, 40.0.deg.inRad))
                pose("spike1End", Pose(113.0, 85.0, 0.0.deg.inRad))
                pose("spike2End", Pose(113.0, 60.0, 0.0.deg.inRad))
                pose("spike2Ctrl", Pose(95.0, 58.0, 0.0.deg.inRad))
                pose("spike3Start", Pose(105.0, 36.0, 0.0.deg.inRad))
                pose("spike3End", Pose(120.0, 36.0, 0.0.deg.inRad))
                pose("spike3Ctrl", Pose(90.0, 35.0, 0.0.deg.inRad))
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
                angle("middleTurretRed", (-132.0).deg)
                angle("middleTurretBlue", 132.0.deg)
                angle("parkTurretRed", (-90.0).deg)
                angle("parkTurretBlue", 90.0.deg)
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
                "startNearToShootMiddle",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("startNear"), p("shootMiddle")))
                    .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                            HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.1,
                                HeadingInterpolator.constant(p("startNear").heading),
                            ),
                            HeadingInterpolator.PiecewiseNode(
                                0.1,
                                0.5,
                                HeadingInterpolator.tangent,
                            ),
                            HeadingInterpolator.PiecewiseNode(
                                0.5,
                                1.0,
                                HeadingInterpolator.constant(p("shootMiddle").heading)
                            ),
                        )
                    )
                    .build(),
            )
            path(
                "shootMiddleToSpike1",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("shootMiddle"), p("spike1End")))
                    .setLinearHeadingInterpolation(p("shootMiddle").heading, p("spike1End").heading)
                    .build(),
            )
            path(
                "spike1Combined",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("shootMiddle"), p("spike1End")))
                    .setLinearHeadingInterpolation(p("shootMiddle").heading, p("spike1End").heading)
                    .addPath(BezierLine(p("spike1End"), p("shootMiddle")))
                    .setLinearHeadingInterpolation(p("spike1End").heading, p("shootMiddle").heading)
                    .build(),
            )
            path(
                "spike2Combined",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootMiddle"), p("spike2Ctrl"), p("spike2End")))
                    .setLinearHeadingInterpolation(p("shootMiddle").heading, p("spike2End").heading)
                    .addPath(BezierLine(p("spike2End"), p("shootMiddle")))
                    .setLinearHeadingInterpolation(p("spike2End").heading, p("shootMiddle").heading)
                    .build(),
            )
            path(
                "spike3Combined",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootMiddle"), p("spike3Ctrl"), p("spike3Start")))
                    .setLinearHeadingInterpolation(p("shootMiddle").heading, p("spike3End").heading)
                    .addPath(BezierLine(p("spike3Start"), p("spike3End")))
                    .setLinearHeadingInterpolation(p("spike3End").heading, p("shootMiddle").heading)
                    .addPath(BezierLine(p("spike3End"), p("shootPark")))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build(),
            )
            path(
                "spike1ToShootMiddle",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("spike1End"), p("shootMiddle")))
                    .setLinearHeadingInterpolation(p("spike1End").heading, p("shootMiddle").heading)
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
            path(
                "shootNearSideSpike2",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootNear"), p("sideSpike2Ctrl"), p("sideSpike2")))
                    .setConstantHeadingInterpolation(p("startNear").heading)
                    .addPath(BezierLine(p("sideSpike2"), p("shootMiddle")))
                    .setConstantHeadingInterpolation(p("shootMiddle").heading)
                    .build(),
            )
            path(
                "shootMiddleGateIntake",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootMiddle"),p("gateIntakeCtrl"), p("gateIntake")))
                    .setLinearHeadingInterpolation(
                        p("shootMiddle").heading,
                        p("gateIntake").heading,
                    )
                    .build(),
            )
            path(
                "gateIntakeShootMiddle",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("gateIntake"), p("shootMiddle")))
                    .setLinearHeadingInterpolation(
                        p("gateIntake").heading,
                        p("shootMiddle").heading,
                    )
                    .build(),
            )
        }
    }

    private fun MutableMap<String, PathChain>.path(name: String, path: PathChain) {
        this[name] = path
    }
}
