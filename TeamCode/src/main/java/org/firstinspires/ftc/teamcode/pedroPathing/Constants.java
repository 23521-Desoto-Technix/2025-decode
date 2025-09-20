package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

  public static FollowerConstants followerConstants = new FollowerConstants()
    .mass(13)
    .forwardZeroPowerAcceleration(-41.278)
    .lateralZeroPowerAcceleration(-59.7819)
    .useSecondaryTranslationalPIDF(false)
    .useSecondaryHeadingPIDF(false)
    .useSecondaryDrivePIDF(false)
    .centripetalScaling(0.0005)
    .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
    .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0))
    .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0, 0.6, 0));

  public static MecanumConstants driveConstants = new MecanumConstants()
    .leftFrontMotorName("leftFront")
    .leftRearMotorName("leftRear")
    .rightFrontMotorName("rightFront")
    .rightRearMotorName("rightRear")
    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
    .xVelocity(57.8741)
    .yVelocity(52.295);

  public static ThreeWheelConstants localizerConstants =
    new ThreeWheelConstants()
      .forwardTicksToInches(.001989436789)
      .strafeTicksToInches(.001989436789)
      .turnTicksToInches(.001989436789)
      .leftPodY(1)
      .rightPodY(-1)
      .strafePodX(-2.5)
      .leftEncoder_HardwareMapName("leftFront")
      .rightEncoder_HardwareMapName("rightRear")
      .strafeEncoder_HardwareMapName("rightFront")
      .leftEncoderDirection(Encoder.REVERSE)
      .rightEncoderDirection(Encoder.REVERSE)
      .strafeEncoderDirection(Encoder.FORWARD);

  public static PathConstraints pathConstraints = new PathConstraints(
    0.995,
    500,
    1,
    1
  );

  public static Follower createFollower(HardwareMap hardwareMap) {
    return new FollowerBuilder(followerConstants, hardwareMap)
      .mecanumDrivetrain(driveConstants)
      .threeWheelLocalizer(localizerConstants)
      .pathConstraints(pathConstraints)
      .build();
  }
}
