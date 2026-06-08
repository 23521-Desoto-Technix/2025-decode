package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OctoQuadConstants;
import com.pedropathing.ftc.localization.localizers.OctoQuadLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.97)
            .centripetalScaling(0.0005)
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.25, 0.08226, 0.001326));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(75.0)
            .yVelocity(58.6)
            .useBrakeModeInTeleOp(true);

    public static OctoQuadConstants localizerConstants =
            new OctoQuadConstants()
                    .name("octoquad")
                    .deadwheelPortX(0)
                    .deadwheelPortY(1)
                    .imuScalar(1)
                    .deadwheelXTicksPerMM(9.9471F)
                    .deadwheelYTicksPerMM(9.9471F)
                    .deadwheelXDir(OctoQuad.EncoderDirection.FORWARD)
                    .deadwheelYDir(OctoQuad.EncoderDirection.FORWARD)
                    .tcpOffsetXMM(-89.154F)
                    .tcpOffsetYMM(-74.168F);

            /*new PinpointConstants()
                    .hardwareMapName("pinpoint")
                    .strafePodX(-3.51)
                    .forwardPodY(-2.92)
                    .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
                    .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                    .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);*/

    public static PathConstraints pathConstraints = new PathConstraints(
            0.97,
            100,
            1.25,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        OctoQuadLocalizer localizer = new OctoQuadLocalizer(hardwareMap, localizerConstants, OctoQuadLocalizer.InitMode.INITIALIZE_OCTOQUAD);

        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .setLocalizer(localizer)
                .pathConstraints(pathConstraints)
                .build();
    }
}
