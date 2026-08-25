package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.7)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.067,
                    0,
                    0.004,
                    0.033))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.94,
                    0,
                    0.01,
                    0.006))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.015,
                    0.0,
                    0,
                    0.6,
                    0.003))
            .centripetalScaling(0.0007)
            .forwardZeroPowerAcceleration(-32.7205683333)
            .lateralZeroPowerAcceleration(-87.7003368793)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1.1,
            1)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.2)
            .strafePodX(3.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static SwerveConstants driveConstants = new SwerveConstants()
            .maxPower(1)
            .useBrakeModeInTeleOp(true)
            .zeroPowerBehavior(SwerveConstants.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES)
            .velocity(67.0)
            ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .swerveDrivetrain(
                        driveConstants,
                        frontLeftPod(hardwareMap),
                        frontRightPod(hardwareMap),
                        backLeftPod(hardwareMap),
                        backRightPod(hardwareMap))
                .build();
    }

    private static PatchedCoaxialPod frontLeftPod(HardwareMap hardwareMap) {
        return new PatchedCoaxialPod(
                hardwareMap,
                "frontLeft",
                "frontLeft1",
                "frontLeft2",
                new PIDFCoefficients(0.35, 0.0, 0.0, 0.12),
                DcMotorSimple.Direction.FORWARD,
                CRServo.Direction.FORWARD,
                Math.toRadians(78.1),
                new Pose(7.0, 7.0, 0.0),
                0.0,
                3.3,
                true,
                240.0);
    }

    private static PatchedCoaxialPod frontRightPod(HardwareMap hardwareMap) {
        return new PatchedCoaxialPod(
                hardwareMap,
                "frontRight",
                "frontRight1",
                "frontRight2",
                new PIDFCoefficients(0.35, 0.0, 0.0, 0.12),
                DcMotorSimple.Direction.FORWARD,
                CRServo.Direction.FORWARD,
                Math.toRadians(240.2),
                new Pose(7.0, -7.0, 0.0),
                0.0,
                3.3,
                true,
                240.0);
    }

    private static PatchedCoaxialPod backLeftPod(HardwareMap hardwareMap) {
        return new PatchedCoaxialPod(
                hardwareMap,
                "backLeft",
                "backLeft1",
                "backLeft2",
                new PIDFCoefficients(0.35, 0.0, 0.0, 0.12),
                DcMotorSimple.Direction.FORWARD,
                CRServo.Direction.FORWARD,
                Math.toRadians(10.4),
                new Pose(-7.0, 7.0, 0.0),
                0.0,
                3.3,
                true,
                240.0);
    }

    private static PatchedCoaxialPod backRightPod(HardwareMap hardwareMap) {
        return new PatchedCoaxialPod(
                hardwareMap,
                "backRight",
                "backRight1",
                "backRight2",
                new PIDFCoefficients(0.35, 0.0, 0.0, 0.12),
                DcMotorSimple.Direction.FORWARD,
                CRServo.Direction.FORWARD,
                Math.toRadians(77.9),
                new Pose(-7.0, -7.0, 0.0),
                0.0,
                3.3,
                true,
                240.0);
    }
}
