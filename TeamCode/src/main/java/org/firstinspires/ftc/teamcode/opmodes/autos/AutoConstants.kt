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

object AutoConstants {
    object Poses {
        // Only add entries for poses that should not use automatic mirroring.
        private val bluePoseOverrides = linkedMapOf<String, Pose>()

        private val redPoses =
            linkedMapOf<String, Pose>().apply {
                pose("startNear", Pose(124.88, 123.83, -143.7.deg.inRad))
                pose("startFar", Pose(80.51, 9.72, 0.0.deg.inRad))
                pose("shootNear", Pose(110.0, 98.0, -90.0.deg.inRad))
                pose("shootMiddle", Pose(88.0, 80.0, 0.0.deg.inRad))
                pose("shootFar", Pose(85.0, 22.0, 0.0.deg.inRad))
                pose("parkFar", Pose(95.0, 22.0, 0.0.deg.inRad))
                pose("parkNear", Pose(100.0, 74.0, 0.0.deg.inRad))
                pose("push", Pose(90.0, 8.5, 0.0.deg.inRad))
                pose("shootPark", Pose(85.0, 105.0, 0.0.deg.inRad))
                pose("sideSpike1", Pose(119.0, 88.0, -90.0.deg.inRad))
                pose("sideSpike1Ctrl", Pose(120.0, 100.0, -90.0.deg.inRad))
                pose("sideSpike2", Pose(121.0, 74.0, -90.0.deg.inRad))
                pose("sideSpike2Ctrl", Pose(121.0, 93.0, -90.0.deg.inRad))
                pose("sideSpike3", Pose(125.0, 15.0, -90.0.deg.inRad))
                pose("sideSpike3TransitionalCtrl", Pose(118.0, 45.0, -90.0.deg.inRad))
                pose("sideSpike3Ctrl", Pose(121.0, 58.0, -90.0.deg.inRad))
                pose("gateHit", Pose(127.0, 62.0, -90.0.deg.inRad))
                pose("gateHitCtrl", Pose(120.0, 58.0, -90.0.deg.inRad))
                pose("gateIntake", Pose(132.0, 58.0, 25.0.deg.inRad))
                pose(
                    "gateIntakeB",
                    Pose(131.81, 59.0, 30.0.deg.inRad),
                    Pose(8.0, 60.82, 150.0.deg.inRad),
                )
                pose(
                    "gateIntakeC",
                    Pose(131.81, 57.06, 30.0.deg.inRad),
                    Pose(8.0, 57.82, 150.0.deg.inRad),
                )
                pose(
                    "gateIntakeD",
                    Pose(131.81, 57.06, 30.0.deg.inRad),
                    Pose(10.0, 59.32, 150.0.deg.inRad),
                )
                pose("gateIntakeCtrl", Pose(105.0, 58.0, 40.0.deg.inRad))
                pose("spike1End", Pose(113.0, 85.0, 0.0.deg.inRad))
                pose("spike2End", Pose(120.0, 58.0, 0.0.deg.inRad))
                pose("spike2Gate", Pose(129.0, 61.0, 0.0.deg.inRad))
                pose("spike2Ctrl", Pose(95.0, 56.0, 0.0.deg.inRad))
                pose("spike2CtrlGate", Pose(97.0, 49.0, 0.0.deg.inRad))
                pose("spike2CtrlFar", Pose(97.0, 49.0, 0.0.deg.inRad))
                pose("sotmCtrl1", Pose(88.0, 95.0, 0.0.deg.inRad))
                pose("sotmCtrl2", Pose(82.0, 60.0, 0.0.deg.inRad))
                pose("spike3Start", Pose(105.0, 36.0, 0.0.deg.inRad))
                pose("spike3End", Pose(120.0, 36.0, 0.0.deg.inRad))
                pose("spike3Wall", Pose(130.0, 36.0, 0.0.deg.inRad))
                pose("spike3CtrlFarToNear", Pose(95.0, 50.0, 180.0.deg.inRad))
                pose("wallEndA", Pose(134.5, 36.0, 0.0.deg.inRad))
                pose("wallEndB", Pose(134.5, 30.0, 0.0.deg.inRad))
                pose("wallEndC", Pose(134.5, 24.0, 0.0.deg.inRad))
                pose("wallEndD", Pose(134.5, 18.0, 0.0.deg.inRad))
                pose("wallEndE", Pose(134.5, 12.0, 0.0.deg.inRad))
                pose("wallEndF", Pose(134.5, 12.0, 0.0.deg.inRad))
                pose("wallSweep", Pose(132.0, 47.0, 0.0.deg.inRad))
                pose("wallSweepCtrl", Pose(132.0, 22.0, 0.0.deg.inRad))
                pose("spike3Ctrl", Pose(90.0, 35.0, 0.0.deg.inRad))
                pose("spike3CtrlFar", Pose(85.0, 35.0, 0.0.deg.inRad))
                pose("humanIntake", Pose(133.0, 9.5, 0.0.deg.inRad))
                pose("humanIntakeCtrl", Pose(89.0, 9.5, 0.0.deg.inRad))
            }

        val red: Map<String, Pose>
            get() = redPoses

        val blue: Map<String, Pose> by lazy {
            redPoses.mapValues { (name, redPose) -> bluePoseOverrides[name] ?: redPose.mirror() }
        }

        fun forAlliance(alliance: Alliance): Map<String, Pose> {
            return if (alliance == Alliance.BLUE) blue else red
        }

        private fun MutableMap<String, Pose>.pose(
            name: String,
            redPose: Pose,
            bluePoseOverride: Pose? = null,
        ) {
            this[name] = redPose
            if (bluePoseOverride != null) {
                bluePoseOverrides[name] = bluePoseOverride
            }
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
                angle("closeTurretRed", (-30.0).deg)
                angle("closeTurretBlue", 30.0.deg)
                angle("middleTurretRed", (-132.0).deg)
                angle("middleTurretBlue", 132.0.deg)
                angle("parkTurretRed3", (-90.0).deg)
                angle("parkTurretBlue3", 90.0.deg)
                angle("parkTurretRed1", (-112.0).deg)
                angle("parkTurretBlue1", 112.0.deg)
                angle("farTurretRed", (-114.0).deg)
                angle("farTurretBlue", 116.0.deg)
                angle("startTurretRed", (0.0).deg)
                angle("startTurretBlue", 0.0.deg)
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
        fun x(name: String) = p(name).x
        fun y(name: String) = p(name).y
        fun h(name: String) = p(name).heading

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
                    .setTangentHeadingInterpolation()
                    .build(),
            )
            path(
                "shootMiddleToParkNear",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("shootMiddle"), p("parkNear")))
                    .setTangentHeadingInterpolation()
                    .build(),
            )
            path(
                "startNearToSpike2",
                follower
                    .pathBuilder()
                    .addPath(
                        BezierCurve(p("startNear"), p("sotmCtrl1"), p("sotmCtrl2"), p("spike2End"))
                    )
                    .setTangentHeadingInterpolation()
                    .build(),
            )
            path(
                "startNearToSpike2Gate",
                follower
                    .pathBuilder()
                    .addPath(
                        BezierCurve(p("startNear"), p("sotmCtrl1"), p("sotmCtrl2"), p("spike2Gate"))
                    )
                    .setTangentHeadingInterpolation()
                    .build(),
            )
            path(
                "spike2ToShootMiddle",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("spike2End"), p("shootMiddle")))
                    .setTangentHeadingInterpolation()
                    .setReversed()
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
                    .setConstantHeadingInterpolation(h("shootMiddle"))
                    .addPath(BezierLine(p("spike1End"), p("shootMiddle")))
                    .setConstantHeadingInterpolation(h("shootMiddle"))
                    .build(),
            )
            path(
                "spike1FastPark",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("shootMiddle"), p("spike1End")))
                    .setLinearHeadingInterpolation(p("shootMiddle").heading, p("spike1End").heading)
                    .addPath(BezierLine(p("spike1End"), p("shootPark")))
                    .setReversed()
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
                "spike2CombinedFarToNear",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootFar"), p("spike2CtrlFar"), p("spike2End")))
                    .setLinearHeadingInterpolation(p("shootMiddle").heading, p("spike2End").heading)
                    .addPath(BezierLine(p("spike2End"), p("shootMiddle")))
                    .setLinearHeadingInterpolation(p("spike2End").heading, p("shootMiddle").heading)
                    .build(),
            )
            path(
                "spike2GateHit",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootMiddle"), p("spike2CtrlGate"), p("gateHit")))
                    .setLinearHeadingInterpolation(p("shootMiddle").heading, p("spike2End").heading)
                    .build(),
            )
            path(
                "gateHitToShootMiddle",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("gateHit"), p("shootMiddle")))
                    .setLinearHeadingInterpolation(p("shootMiddle").heading, p("spike2End").heading)
                    .build(),
            )
            path(
                "spike3Combined",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootMiddle"), p("spike3Ctrl"), p("spike3Start")))
                    .setTangentHeadingInterpolation()
                    .addPath(BezierLine(p("spike3Start"), p("spike3End")))
                    .setTangentHeadingInterpolation()
                    .addPath(BezierLine(p("spike3End"), p("shootPark")))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build(),
            )
            path(
                "sideSpike3Combined",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("shootMiddle"), p("sideSpike3")))
                    .setTangentHeadingInterpolation()
                    .addPath(BezierLine(p("spike3End"), p("shootPark")))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build(),
            )
            path(
                "spike3CombinedFar",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootMiddle"), p("spike3Ctrl"), p("spike3Start")))
                    .setConstantHeadingInterpolation(p("shootMiddle").heading)
                    .addPath(BezierLine(p("spike3Start"), p("spike3End")))
                    .setConstantHeadingInterpolation(p("shootMiddle").heading)
                    .addPath(BezierLine(p("spike3End"), p("shootFar")))
                    .setConstantHeadingInterpolation(p("shootMiddle").heading)
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
                    .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                            HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.2,
                                HeadingInterpolator.constant(p("startNear").heading),
                            ),
                            HeadingInterpolator.PiecewiseNode(
                                0.2,
                                1.0,
                                HeadingInterpolator.constant(p("shootMiddle").heading),
                            ),
                        )
                    )
                    .build(),
            )
            path(
                "shootMiddleGateIntake",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootMiddle"), p("gateIntakeCtrl"), p("gateIntake")))
                    .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                            HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.5,
                                HeadingInterpolator.tangent,
                            ),

                            HeadingInterpolator.PiecewiseNode(
                                0.5,
                                1.0,
                                HeadingInterpolator.constant(h("gateIntake")),
                            ),
                        )
                    )
                    .setGlobalDeceleration()
                    .setBrakingStart(100.0)
                    .build(),
            )
            path(
                "shootMiddleGateIntakeB",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootMiddle"), p("gateIntakeCtrl"), p("gateIntakeB")))
                    .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                            HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.55,
                                HeadingInterpolator.constant(h("shootMiddle")),
                            ),
                            HeadingInterpolator.PiecewiseNode(
                                0.55,
                                1.0,
                                HeadingInterpolator.constant(h("gateIntakeB")),
                            ),
                        )
                    )
                    .build(),
            )
            path(
                "shootMiddleGateIntakeC",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootMiddle"), p("gateIntakeCtrl"), p("gateIntakeC")))
                    .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                            HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.55,
                                HeadingInterpolator.constant(h("shootMiddle")),
                            ),
                            HeadingInterpolator.PiecewiseNode(
                                0.55,
                                1.0,
                                HeadingInterpolator.constant(h("gateIntakeC")),
                            ),
                        )
                    )
                    .build(),
            )
            path(
                "shootMiddleGateIntakeD",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootMiddle"), p("gateIntakeCtrl"), p("gateIntakeD")))
                    .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                            HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.55,
                                HeadingInterpolator.constant(h("shootMiddle")),
                            ),
                            HeadingInterpolator.PiecewiseNode(
                                0.55,
                                1.0,
                                HeadingInterpolator.constant(h("gateIntakeD")),
                            ),
                        )
                    )
                    .build(),
            )
            path(
                "gateIntakeShootMiddle",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("gateIntake"), p("shootMiddle")))
                    .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                            HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.1,
                                HeadingInterpolator.constant(h("gateIntake")),
                            ),
                            HeadingInterpolator.PiecewiseNode(
                                0.1,
                                0.15,
                                HeadingInterpolator.linear(h("gateIntake"), h("shootMiddle")),
                            ),
                            HeadingInterpolator.PiecewiseNode(
                                0.15,
                                1.0,
                                HeadingInterpolator.tangent.reverse(),
                            ),
                        )
                    )
                    .build(),
            )
            path(
                "shootMiddleToGateHit",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("shootMiddle"), p("gateHit")))
                    .setConstantHeadingInterpolation(p("gateHit").heading)
                    .build(),
            )
            path(
                "gateHitSideSpike3",
                follower
                    .pathBuilder()
                    .addPath(
                        BezierCurve(p("gateHit"), p("sideSpike3TransitionalCtrl"), p("sideSpike3"))
                    )
                    .setConstantHeadingInterpolation(p("gateHit").heading)
                    .addPath(BezierLine(p("sideSpike3"), p("shootFar")))
                    .setLinearHeadingInterpolation(p("sideSpike3").heading, p("shootFar").heading)
                    .build(),
            )
            path(
                "shootMiddleSideSpike3",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootMiddle"), p("sideSpike3Ctrl"), p("sideSpike3")))
                    .setConstantHeadingInterpolation(p("gateHit").heading)
                    .addPath(BezierLine(p("sideSpike3"), p("shootFar")))
                    .setLinearHeadingInterpolation(p("sideSpike3").heading, p("shootFar").heading)
                    .build(),
            )
            path(
                "shootFarHumanIntake",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootFar"), p("humanIntakeCtrl"), p("humanIntake")))
                    .setConstantHeadingInterpolation(p("humanIntake").heading)
                    .addPath(BezierLine(p("humanIntake"), p("shootFar")))
                    .setConstantHeadingInterpolation(p("humanIntake").heading)
                    .build(),
            )
            path(
                "shootFarPark",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("shootFar"), p("parkFar")))
                    .setConstantHeadingInterpolation(p("parkFar").heading)
                    .build(),
            )
            path(
                "startFarPush",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("startFar"), p("push")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "startFarToShootFar",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("startFar"), p("shootFar")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "shootFarToHumanIntake",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootFar"), p("humanIntakeCtrl"), p("humanIntake")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "humanIntakeToShootFar",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("humanIntake"), p("shootFar")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "shootFarToSpike3",
                follower
                    .pathBuilder()
                    .addPath(BezierCurve(p("shootFar"), p("spike3CtrlFar"), p("spike3End")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "shootFarToWallIntakeA",
                follower
                    .pathBuilder()
                    .addPath(
                        BezierCurve(
                            p("shootFar"),
                            Pose(x("shootFar"), y("wallEndA")),
                            p("wallEndA"),
                        )
                    )
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "wallIntakeAToShootFar",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("wallEndA"), p("shootFar")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "shootFarToWallIntakeB",
                follower
                    .pathBuilder()
                    .addPath(
                        BezierCurve(
                            p("shootFar"),
                            Pose(x("shootFar"), y("wallEndB")),
                            p("wallEndB"),
                        )
                    )
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "wallIntakeBToShootFar",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("wallEndB"), p("shootFar")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "shootFarToWallIntakeC",
                follower
                    .pathBuilder()
                    .addPath(
                        BezierCurve(
                            p("shootFar"),
                            Pose(x("shootFar"), y("wallEndC")),
                            p("wallEndC"),
                        )
                    )
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "wallIntakeCToShootFar",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("wallEndC"), p("shootFar")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "shootFarToWallIntakeD",
                follower
                    .pathBuilder()
                    .addPath(
                        BezierCurve(
                            p("shootFar"),
                            Pose(x("shootFar"), y("wallEndD")),
                            p("wallEndD"),
                        )
                    )
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "wallIntakeDToShootFar",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("wallEndD"), p("shootFar")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "shootFarToWallIntakeE",
                follower
                    .pathBuilder()
                    .addPath(
                        BezierCurve(
                            p("shootFar"),
                            Pose(x("shootFar"), y("wallEndE")),
                            p("wallEndE"),
                        )
                    )
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "wallIntakeEToShootFar",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("wallEndE"), p("shootFar")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "shootFarToWallIntakeF",
                follower
                    .pathBuilder()
                    .addPath(
                        BezierCurve(
                            p("shootFar"),
                            Pose(x("shootFar"), y("wallEndF")),
                            p("wallEndF"),
                        )
                    )
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "wallIntakeFToShootFar",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("wallEndF"), p("shootFar")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "shootFarToWallSweep",
                follower
                    .pathBuilder()
                    .addPath(
                        BezierCurve(
                            p("shootFar"),
                            p("wallSweepCtrl"),
                            p("wallSweep"),
                        )
                    )
                    .setTangentHeadingInterpolation()
                    .build(),
            )
            path(
                "wallSweepToShootFar",
                follower
                    .pathBuilder()
                    .addPath(
                        BezierLine(
                            p("wallSweep"),
                            p("shootFar"),
                        )
                    )
                    .setConstantHeadingInterpolation(h("shootFar"))
                    .build(),
            )
            path(
                "spike3ToShootFar",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("spike3End"), p("shootFar")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "spike3ToShootMiddle",
                follower
                    .pathBuilder()
                    .addPath(
                        BezierCurve(p("spike3End"), p("spike3CtrlFarToNear"), p("shootMiddle"))
                    )
                    .setHeadingInterpolation(
                        HeadingInterpolator.piecewise(
                            HeadingInterpolator.PiecewiseNode(
                                0.0,
                                0.7,
                                HeadingInterpolator.tangent,
                            ),
                            HeadingInterpolator.PiecewiseNode(
                                0.7,
                                1.0,
                                HeadingInterpolator.constant(h("spike3CtrlFarToNear")),
                            ),
                        )
                    )
                    .setReversed()
                    .build(),
            )
            path(
                "pushToShootFar",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("push"), p("shootFar")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
            path(
                "shootFarToParkFar",
                follower
                    .pathBuilder()
                    .addPath(BezierLine(p("shootFar"), p("parkFar")))
                    .setConstantHeadingInterpolation(p("startFar").heading)
                    .build(),
            )
        }
    }

    private fun MutableMap<String, PathChain>.path(name: String, path: PathChain) {
        this[name] = path
    }
}
