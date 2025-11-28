package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
public class Constants {

    public static PIDFCoefficients translationalPIDF = new PIDFCoefficients(0.2,0,0.01,0);
    public static PIDFCoefficients headingPIDF = new PIDFCoefficients(1.3,0,0.01,0);
    public static FilteredPIDFCoefficients drivePIDF = new FilteredPIDFCoefficients(0.1,0,0.01,0,0);
    public static double centripetalScale = 0.005;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-56.640604422602244)
            .lateralZeroPowerAcceleration(-69.32544024733782)
            .translationalPIDFCoefficients(translationalPIDF)
            .headingPIDFCoefficients(headingPIDF)
            .drivePIDFCoefficients(drivePIDF)
            .centripetalScaling(centripetalScale)
            .mass(9.389);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.05, 0.1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(72.31624887393829)
            .yVelocity(56.99649765973459)
            .rightFrontMotorName("FrontRight")
            .rightRearMotorName("RearRight")
            .leftRearMotorName("RearLeft")
            .leftFrontMotorName("FrontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.0019536877479381168)
            .strafeTicksToInches(.001959674181210116)
            .turnTicksToInches(.0020001324500057574)
            .leftPodY(3.25)
            .rightPodY(-3.25)
            .strafePodX(-0.5)
            .leftEncoder_HardwareMapName("RearRight")
            .rightEncoder_HardwareMapName("RearLeft")
            .strafeEncoder_HardwareMapName("FrontLeft")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}