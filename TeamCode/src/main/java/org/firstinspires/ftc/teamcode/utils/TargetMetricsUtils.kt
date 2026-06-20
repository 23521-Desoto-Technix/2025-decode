package org.firstinspires.ftc.teamcode.utils

import com.pedropathing.geometry.Pose
import com.pedropathing.math.Vector
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import org.firstinspires.ftc.teamcode.utils.PoseUtils.mirrorPose
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

data class TargetMetrics(val distanceToTarget: Double, val relativeAngleToTarget: Angle)

private const val TARGETING_LOCAL_X = -1.633
private const val TARGETING_LOCAL_Y = 0.0

fun applyRobotSpaceOffset(pose: Pose, localX: Double, localY: Double): Pose {
    val heading = pose.heading
    val fieldX = pose.x + localX * cos(heading) - localY * sin(heading)
    val fieldY = pose.y + localX * sin(heading) + localY * cos(heading)
    return Pose(fieldX, fieldY, pose.heading)
}

fun calculateTargetingPose(robotPose: Pose): Pose {
    return applyRobotSpaceOffset(robotPose, TARGETING_LOCAL_X, TARGETING_LOCAL_Y)
}

fun calculateTargetMetrics(
    robotPose: Pose,
    angularVelocity: Double = 0.0,
    velocity: Vector,
): TargetMetrics {
    val currentPose = calculateTargetingPose(robotPose)

    val targetPose =
        if (currentPose.y < 48.0) {
            if (BotState.alliance == Alliance.BLUE) {
                Pose(4.0, 140.0, 0.0)
            } else {
                Pose(140.0, 140.0, 0.0)
            }
        } else {
            if (BotState.alliance == Alliance.BLUE) {
                Pose(0.0, 144.0, 0.0)
            } else {
                Pose(144.0, 144.0, 0.0)
            }
        }

    val projectedVelocity =
        velocity.times(
            400.milliseconds.inWholeMicroseconds.toDouble() /
                1.seconds.inWholeMicroseconds.toDouble()
        )

    val currentX = currentPose.x + projectedVelocity.xComponent
    val currentY = currentPose.y + projectedVelocity.yComponent
    val deltaX = targetPose.x - currentX
    val deltaY = targetPose.y - currentY
    val distanceToTarget = hypot(deltaX, deltaY)

    val redAnglePoseA = Pose(141.5, 130.0, 0.0)
    val redAnglePoseB = Pose(120.0, 141.5, 0.0)
    val anglePoseA = if (BotState.alliance == Alliance.BLUE) mirrorPose(redAnglePoseA) else redAnglePoseA
    val anglePoseB = if (BotState.alliance == Alliance.BLUE) mirrorPose(redAnglePoseB) else redAnglePoseB
    val angleToPoseA = atan2(anglePoseA.y - currentY, anglePoseA.x - currentX)
    val angleToPoseB = atan2(anglePoseB.y - currentY, anglePoseB.x - currentX)
    val absoluteAngleToTarget =
        atan2(sin(angleToPoseA) + sin(angleToPoseB), cos(angleToPoseA) + cos(angleToPoseB)).rad
    val predictiveHeading =
        currentPose.heading.rad +
            (angularVelocity / 1.seconds.inWholeMicroseconds *
                    90.milliseconds.inWholeMicroseconds)
                .rad
    val relativeAngleToTarget = (predictiveHeading - absoluteAngleToTarget + 180.deg).normalized

    return TargetMetrics(distanceToTarget, relativeAngleToTarget)
}

