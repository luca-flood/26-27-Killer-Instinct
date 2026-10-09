package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;

public class Constants {
    public static final double WHEEL_DEGREES_PER_ENCODER_REVOLUTION = 240.0;

    // Latest calibrated raw encoder angles with all wheels physically facing forward.
    public static final double FRONT_LEFT_STRAIGHT_ENCODER_DEG = 150.244;
    public static final double BACK_LEFT_STRAIGHT_ENCODER_DEG = 99.187;
    public static final double FRONT_RIGHT_STRAIGHT_ENCODER_DEG = 57.146;
    public static final double BACK_RIGHT_STRAIGHT_ENCODER_DEG = 99.919;

    // Assumes steering axes lie on wheel centerlines and the robot origin is their rectangle's center.
    public static final double POD_TRACK_WIDTH_IN = 15.0 - 20.0 / 25.4;
    // Assumes the reported 13.2 inches is front-to-back steering-axis spacing.
    public static final double POD_WHEELBASE_IN = 13.2;
    public static final double FRONT_LEFT_X_IN = POD_WHEELBASE_IN / 2.0;
    public static final double FRONT_LEFT_Y_IN = POD_TRACK_WIDTH_IN / 2.0;
    public static final double FRONT_RIGHT_X_IN = POD_WHEELBASE_IN / 2.0;
    public static final double FRONT_RIGHT_Y_IN = -POD_TRACK_WIDTH_IN / 2.0;
    public static final double BACK_LEFT_X_IN = -POD_WHEELBASE_IN / 2.0;
    public static final double BACK_LEFT_Y_IN = POD_TRACK_WIDTH_IN / 2.0;
    public static final double BACK_RIGHT_X_IN = -POD_WHEELBASE_IN / 2.0;
    public static final double BACK_RIGHT_Y_IN = -POD_TRACK_WIDTH_IN / 2.0;

    // TODO: Tune these with Pedro/Panels tuning once pod angle tracking is verified.
    public static double TURN_P = 0.3;
    public static double TURN_I = 0.0;
    public static double TURN_D = 0.03;
    public static double TURN_F = 0.05;

    // TODO: Confirm motor, servo, and encoder directions on the real robot.
    public static final DcMotorSimple.Direction DEFAULT_DRIVE_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction DEFAULT_SERVO_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final boolean DEFAULT_ENCODER_REVERSED = true;

    // Measured bidirectional voltage bounds and straight offsets share the same calibrated range.
    public static final PodCalibration frontLeft = new PodCalibration(FRONT_LEFT_STRAIGHT_ENCODER_DEG, 0.034, 3.228);
    public static final PodCalibration frontRight = new PodCalibration(FRONT_RIGHT_STRAIGHT_ENCODER_DEG, 0.036, 3.211);
    public static final PodCalibration backLeft = new PodCalibration(BACK_LEFT_STRAIGHT_ENCODER_DEG, 0.028, 3.251);
    public static final PodCalibration backRight = new PodCalibration(BACK_RIGHT_STRAIGHT_ENCODER_DEG, 0.036, 3.239);

    static {
        frontRight.driveDirection = DcMotorSimple.Direction.FORWARD;
        backLeft.driveDirection = DcMotorSimple.Direction.REVERSE;
        frontLeft.p = 0.3;
        frontLeft.i = 0.0;
        frontLeft.d = 0.03;
        frontLeft.f = 0.07;
        frontRight.p = 0.36;
        frontRight.i = 0.0;
        frontRight.d = 0.04;
        frontRight.f = 0.07;
        backLeft.p = 0.35;
        backLeft.i = 0.0;
        backLeft.d = 0.04;
        backLeft.f = 0.04;
        backRight.p = 0.3;
        backRight.i = 0.0;
        backRight.d = 0.03;
        backRight.f = 0.05;
    }

    public static class PodCalibration {
        public double analogMinVoltage;
        public double analogMaxVoltage;
        public double straightEncoderDeg;
        public double p = TURN_P;
        public double i = TURN_I;
        public double d = TURN_D;
        public double f = TURN_F;
        public DcMotorSimple.Direction driveDirection = DEFAULT_DRIVE_DIRECTION;
        public DcMotorSimple.Direction servoDirection = DEFAULT_SERVO_DIRECTION;
        public boolean encoderReversed = DEFAULT_ENCODER_REVERSED;

        public PodCalibration(double straightEncoderDeg, double analogMinVoltage, double analogMaxVoltage) {
            this.straightEncoderDeg = straightEncoderDeg;
            this.analogMinVoltage = analogMinVoltage;
            this.analogMaxVoltage = analogMaxVoltage;
        }

        public Controller controller() {
            return Controller.pid(p, i, d).plus(Controller.proportionalFeedforward(f));
        }
    }

    // TODO: Tune Pedro v3 Foresight values after localization and pod steering are reliable.
    public static double HEADING_P = 0.94;
    public static double HEADING_STATIC_F = 0.006;
    public static double FORWARD_TRANSLATIONAL_P = 0.067;
    public static double STRAFE_TRANSLATIONAL_P = 0.067;
    public static double BRAKE_P = 0.015;
    public static double COAST_P = 0.0;
    public static double MAX_PATH_SPEED = 0.7;
    public static double MAX_VELOCITY_CONSTRAINT = 60.0;
    public static double MAX_ACCELERATION_CONSTRAINT = 60.0;
    public static double MAX_DECELERATION_CONSTRAINT = 60.0;
    public static double MAX_ACHIEVABLE_FORWARD_VELOCITY = 60.0;
    public static double MAX_ACHIEVABLE_STRAFE_VELOCITY = 60.0;
    public static double NATURAL_FORWARD_DECELERATION = 30.0;
    public static double NATURAL_STRAFE_DECELERATION = 30.0;

    // TODO: Measure Pinpoint odometry pod offsets from robot center.
    public static double PINPOINT_X_POD_OFFSET_IN = 3.5;
    public static double PINPOINT_Y_POD_OFFSET_IN = -2.2;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    private Constants() {
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, pinpointConfig());

        Swerve drivetrain = createDrivetrain(hardwareMap);

        return new Follower(localizer, drivetrain, new Foresight(foresightConfig()));
    }

    public static Swerve createDrivetrain(HardwareMap hardwareMap) {
        return new Swerve(hardwareMap, swerveConfig(), createPods(hardwareMap));
    }

    public static CoaxialPodConfig[] podConfigs() {
        return new CoaxialPodConfig[] {frontLeftPodConfig(), frontRightPodConfig(),
                backLeftPodConfig(), backRightPodConfig()};
    }

    public static PatchedCoaxialPod[] createPods(HardwareMap hardwareMap) {
        CoaxialPodConfig[] configs = podConfigs();
        PatchedCoaxialPod[] pods = new PatchedCoaxialPod[configs.length];
        for (int i = 0; i < pods.length; i++) {
            pods[i] = new PatchedCoaxialPod(hardwareMap, configs[i], WHEEL_DEGREES_PER_ENCODER_REVOLUTION);
        }
        return pods;
    }

    public static SwerveConfig swerveConfig() {
        return new SwerveConfig(config -> {
            config.manualBrakeMode.set(true);
            config.voltageCompensation.set(true);
            config.nominalVoltage.set(12.0);
            config.staticFrictionCoefficient.set(0.1);
            config.epsilon.set(0.05);
            config.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
        });
    }

    public static CoaxialPodConfig frontLeftPodConfig() {
        return podConfig(
                "frontLeft",
                "frontLeft",
                "frontLeft1",
                "frontLeft2",
                frontLeft,
                Vector2D.cartesian(FRONT_LEFT_X_IN, FRONT_LEFT_Y_IN));
    }

    public static CoaxialPodConfig frontRightPodConfig() {
        return podConfig(
                "frontRight",
                "frontRight",
                "frontRight1",
                "frontRight2",
                frontRight,
                Vector2D.cartesian(FRONT_RIGHT_X_IN, FRONT_RIGHT_Y_IN));
    }

    public static CoaxialPodConfig backLeftPodConfig() {
        return podConfig(
                "backLeft",
                "backLeft",
                "backLeft1",
                "backLeft2",
                backLeft,
                Vector2D.cartesian(BACK_LEFT_X_IN, BACK_LEFT_Y_IN));
    }

    public static CoaxialPodConfig backRightPodConfig() {
        return podConfig(
                "backRight",
                "backRight",
                "backRight1",
                "backRight2",
                backRight,
                Vector2D.cartesian(BACK_RIGHT_X_IN, BACK_RIGHT_Y_IN));
    }

    private static CoaxialPodConfig podConfig(
            String name,
            String motorName,
            String servoName,
            String encoderName,
            PodCalibration calibration,
            Vector2D podOffset) {
        return new CoaxialPodConfig(config -> {
            config.name.set(name);
            config.motorName.set(motorName);
            config.servoName.set(servoName);
            config.servoEncoderName.set(encoderName);
            config.turnController.set(calibration.controller());
            config.driveDirection.set(calibration.driveDirection);
            config.servoDirection.set(calibration.servoDirection);
            config.angleOffsetRad.set(Math.toRadians(calibration.straightEncoderDeg));
            config.podOffset.set(podOffset);
            config.analogMinVoltage.set(calibration.analogMinVoltage);
            config.analogMaxVoltage.set(calibration.analogMaxVoltage);
            config.encoderReversed.set(calibration.encoderReversed);
            config.motorCachingThreshold.set(0.01);
            config.servoCachingThreshold.set(0.01);
        });
    }

    public static Controller turnController() {
        return Controller.sum(
                Controller.pid(TURN_P, TURN_I, TURN_D),
                Controller.staticFeedforward(TURN_F)
        );
    }

    public static PinpointConfig pinpointConfig() {
        return new PinpointConfig(config -> {
            config.name.set("pinpoint");
            config.xPodOffset.set(PINPOINT_X_POD_OFFSET_IN);
            config.yPodOffset.set(PINPOINT_Y_POD_OFFSET_IN);
            config.offsetUnits.set(DistanceUnit.INCH);
            config.globalDistanceUnit.set(DistanceUnit.INCH);
            config.encoderResolutionUnit.set(DistanceUnit.INCH);
            config.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            config.ticksPerUnit.set(OptionalDouble.empty());
            config.xPodDirection.set(PINPOINT_X_DIRECTION);
            config.yPodDirection.set(PINPOINT_Y_DIRECTION);
            config.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU);
        });
    }

    public static ForesightConfig foresightConfig() {
        return new ForesightConfig(config -> {
            config.headingFeedback.set(Controller.proportional(HEADING_P));
            config.headingStaticFF.set(Controller.staticFeedforward(HEADING_STATIC_F));
            config.forwardTranslational.set(Controller.proportional(FORWARD_TRANSLATIONAL_P));
            config.strafeTranslational.set(Controller.proportional(STRAFE_TRANSLATIONAL_P));
            config.brake.set(Controller.proportional(BRAKE_P));
            config.coast.set(Controller.proportional(COAST_P));
            config.holdPointTranslationalScaling.set(1.0);
            config.holdPointHeadingScaling.set(1.0);
            config.maxBrakingPower.set(1.0);
            config.maxAccelerationConstraint.set(MAX_ACCELERATION_CONSTRAINT);
            config.maxVelocityConstraint.set(MAX_VELOCITY_CONSTRAINT);
            config.maxDecelerationConstraint.set(MAX_DECELERATION_CONSTRAINT);
            config.maxPathSpeed.set(MAX_PATH_SPEED);
            config.maxDecelerationScale.set(1.0);
            config.brakeAggression.set(1.0);
            config.coastDownToVelocity.set(0.0);
            config.headingDeviationTolerance.set(Math.toRadians(2.0));
            config.translationalDeviationTolerance.set(1.0);
            config.brakeAtEnd.set(true);
            config.pathSkip.set(false);
            config.headingDriveRatio.set(1.0);
            config.linearBrakeCoefficients.set(Matrix.zero(2));
            config.quadraticBrakeCoefficients.set(Matrix.zero(2));
            config.headingBrakeCoefficients.set(Vector2D.zero());
            config.cosineScale.set(true);
            config.maxAchievableForwardVelocity.set(MAX_ACHIEVABLE_FORWARD_VELOCITY);
            config.maxAchievableStrafeVelocity.set(MAX_ACHIEVABLE_STRAFE_VELOCITY);
            config.naturalForwardDeceleration.set(NATURAL_FORWARD_DECELERATION);
            config.naturalStrafeDeceleration.set(NATURAL_STRAFE_DECELERATION);
            config.minCorrectionDistance.set(1.0);
            config.parametricTConstraint.set(0.995);
            config.headingConstraint.set(Math.toRadians(2.0));
            config.translationalConstraint.set(1.0);
            config.velocityConstraint.set(1.0);
            config.timeoutConstraint.set(0.0);
        });
    }
}
