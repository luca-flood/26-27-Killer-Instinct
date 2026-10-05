package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.SwervePod;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PatchedCoaxialPod implements SwervePod {
    public static final double DEFAULT_WHEEL_DEGREES_PER_ENCODER_REVOLUTION = 240.0;

    private final String name;
    private final AnalogInput turnEncoder;
    private final CRServo turnServo;
    private final DcMotorEx driveMotor;
    private final Supplier<Controller> turnController;
    private final Supplier<DcMotorSimple.Direction> driveDirection;
    private final Supplier<CRServo.Direction> servoDirection;
    private final Supplier<Vector2D> podOffset;
    private final Supplier<Boolean> encoderReversed;
    private final Supplier<Double> motorCachingThreshold;
    private final Supplier<Double> servoCachingThreshold;
    private final Supplier<Double> analogMinVoltage;
    private final Supplier<Double> analogMaxVoltage;
    private final DoubleSupplier wheelAngleRadSupplier;
    private final ScaledAnalogAngleSupplier scaledAngleSupplier;
    private final double wheelToEncoderRatio;

    private double backlashDeadbandRad = Math.toRadians(2.0);
    private double lastDrivePower = 0.0;
    private double lastTurnPower = 0.0;

    public PatchedCoaxialPod(HardwareMap hardwareMap, CoaxialPodConfig config) {
        this(
                hardwareMap,
                config,
                DEFAULT_WHEEL_DEGREES_PER_ENCODER_REVOLUTION);
    }

    public PatchedCoaxialPod(
            HardwareMap hardwareMap,
            CoaxialPodConfig config,
            double wheelDegreesPerEncoderRevolution) {
        this(
                hardwareMap,
                config,
                hardwareMap.get(AnalogInput.class, config.servoEncoderName.get()),
                wheelDegreesPerEncoderRevolution);
    }

    public PatchedCoaxialPod(
            HardwareMap hardwareMap,
            CoaxialPodConfig config,
            AnalogInput turnEncoder,
            double wheelDegreesPerEncoderRevolution) {
        this(
                hardwareMap,
                config.name.get(),
                config.motorName.get(),
                config.servoName.get(),
                turnEncoder,
                config.turnController,
                config.driveDirection,
                config.servoDirection,
                config.podOffset,
                config.encoderReversed,
                config.motorCachingThreshold,
                config.servoCachingThreshold,
                config.analogMinVoltage,
                config.analogMaxVoltage,
                new ScaledAnalogAngleSupplier(
                        turnEncoder,
                        config.analogMinVoltage.get(),
                        config.analogMaxVoltage.get(),
                        config.angleOffsetRad.get(),
                        config.encoderReversed.get(),
                        wheelDegreesPerEncoderRevolution),
                wheelDegreesPerEncoderRevolution);
    }

    public PatchedCoaxialPod(
            HardwareMap hardwareMap,
            CoaxialPodConfig config,
            AnalogInput turnEncoder,
            DoubleSupplier wheelAngleRadSupplier,
            double wheelDegreesPerEncoderRevolution) {
        this(
                hardwareMap,
                config.name.get(),
                config.motorName.get(),
                config.servoName.get(),
                turnEncoder,
                config.turnController,
                config.driveDirection,
                config.servoDirection,
                config.podOffset,
                config.encoderReversed,
                config.motorCachingThreshold,
                config.servoCachingThreshold,
                config.analogMinVoltage,
                config.analogMaxVoltage,
                wheelAngleRadSupplier,
                wheelDegreesPerEncoderRevolution);
    }

    public PatchedCoaxialPod(
            HardwareMap hardwareMap,
            String motorName,
            String servoName,
            String turnEncoderName,
            Controller turnController,
            DcMotorSimple.Direction driveDirection,
            CRServo.Direction servoDirection,
            double angleOffsetRad,
            Vector2D podOffset,
            double analogMinVoltage,
            double analogMaxVoltage,
            boolean encoderReversed,
            double wheelDegreesPerEncoderRevolution) {
        this(
                hardwareMap,
                servoName,
                motorName,
                servoName,
                hardwareMap.get(AnalogInput.class, turnEncoderName),
                () -> turnController,
                () -> driveDirection,
                () -> servoDirection,
                () -> podOffset,
                () -> encoderReversed,
                () -> 0.01,
                () -> 0.01,
                () -> analogMinVoltage,
                () -> analogMaxVoltage,
                new ScaledAnalogAngleSupplier(
                        hardwareMap.get(AnalogInput.class, turnEncoderName),
                        analogMinVoltage,
                        analogMaxVoltage,
                        angleOffsetRad,
                        encoderReversed,
                        wheelDegreesPerEncoderRevolution),
                wheelDegreesPerEncoderRevolution);
    }

    private PatchedCoaxialPod(
            HardwareMap hardwareMap,
            String name,
            String motorName,
            String servoName,
            AnalogInput turnEncoder,
            Supplier<Controller> turnController,
            Supplier<DcMotorSimple.Direction> driveDirection,
            Supplier<CRServo.Direction> servoDirection,
            Supplier<Vector2D> podOffset,
            Supplier<Boolean> encoderReversed,
            Supplier<Double> motorCachingThreshold,
            Supplier<Double> servoCachingThreshold,
            Supplier<Double> analogMinVoltage,
            Supplier<Double> analogMaxVoltage,
            DoubleSupplier wheelAngleRadSupplier,
            double wheelDegreesPerEncoderRevolution) {
        this.name = name;
        this.turnEncoder = turnEncoder;
        this.turnServo = hardwareMap.get(CRServo.class, servoName);
        this.driveMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.turnController = turnController;
        this.driveDirection = driveDirection;
        this.servoDirection = servoDirection;
        this.podOffset = podOffset;
        this.encoderReversed = encoderReversed;
        this.motorCachingThreshold = motorCachingThreshold;
        this.servoCachingThreshold = servoCachingThreshold;
        this.analogMinVoltage = analogMinVoltage;
        this.analogMaxVoltage = analogMaxVoltage;
        this.wheelAngleRadSupplier = wheelAngleRadSupplier;
        this.scaledAngleSupplier = wheelAngleRadSupplier instanceof ScaledAnalogAngleSupplier
                ? (ScaledAnalogAngleSupplier) wheelAngleRadSupplier
                : null;
        this.wheelToEncoderRatio = 360.0 / wheelDegreesPerEncoderRevolution;

        setMotorToFloat();
        driveMotor.setDirection(this.driveDirection.get());
        driveMotor.setPower(0.0);
        turnServo.setDirection(this.servoDirection.get());
        turnServo.setPower(0.0);
    }

    @Override
    public Vector2D getOffset() {
        return podOffset.get();
    }

    @Override
    public double getAngle() {
        return getAngleAfterOffsetRad();
    }

    public void setServoPower(double power) {
        lastTurnPower = power;
        turnServo.setPower(power);
    }

    public void setMotorPower(double power) {
        lastDrivePower = power;
        driveMotor.setPower(power);
    }

    @Override
    public void setToFloat() {
        setMotorToFloat();
    }

    @Override
    public void setToBreak() {
        setMotorToBreak();
    }

    public void setMotorToFloat() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setMotorToBreak() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean isEncoderReversed() {
        return encoderReversed.get();
    }

    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        // Preserve Pedro's servo-frame quarter turn; this supplier already applies encoder reversal.
        return Angle.normalize(wheelTheta + (encoderReversed.get() ? Math.PI / 2 : -Math.PI / 2));
    }

    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        double actualRad = Angle.normalize(getAngleAfterOffsetRad());
        double desiredRad = adjustThetaForEncoder(targetAngleRad);

        double errorRad = Angle.normalizeSigned(desiredRad - actualRad);

        if (Math.abs(errorRad) > (Math.PI / 2.0)) {
            desiredRad = Angle.normalize(desiredRad + Math.PI);
            drivePower = -drivePower;
            errorRad = Angle.normalizeSigned(desiredRad - actualRad);
        }

        double encoderErrorRad = wheelErrorToEncoderError(errorRad);
        double turnPower;
        if (Math.abs(errorRad) < backlashDeadbandRad) {
            turnPower = Utils.clamp(turnController.get().calculate(0.0, encoderErrorRad), -1.0, 1.0);
        } else {
            turnPower = Utils.clamp(
                    turnController.get().calculate(Math.signum(encoderErrorRad), encoderErrorRad),
                    -1.0,
                    1.0);
        }

        double currentServoCachingThreshold = servoCachingThreshold.get();
        double currentMotorCachingThreshold = motorCachingThreshold.get();

        if (ignoreAngleChanges) {
            lastTurnPower = 0.0;
            turnServo.setPower(0.0);
        } else if (Math.abs(turnPower - lastTurnPower) > currentServoCachingThreshold || (turnPower == 0.0 && lastTurnPower != 0.0)) {
            lastTurnPower = turnPower;
            turnServo.setPower(turnPower);
        }

        if (Math.abs(drivePower - lastDrivePower) > currentMotorCachingThreshold || (drivePower == 0.0 && lastDrivePower != 0.0)) {
            lastDrivePower = drivePower;
            driveMotor.setPower(drivePower);
        }
    }

    public double getAngleAfterOffsetRad() {
        return Angle.normalize(wheelAngleRadSupplier.getAsDouble());
    }

    public double getRawAngleRad() {
        double range = getAnalogMaxVoltage() - getAnalogMinVoltage();
        if (range == 0.0) {
            return 0.0;
        }

        double normalized = (turnEncoder.getVoltage() - getAnalogMinVoltage()) / range;
        return Utils.clamp(normalized, 0.0, 1.0) * (2.0 * Math.PI);
    }

    public double getOffsetAngleRad() {
        return Angle.normalize(getAngleAfterOffsetRad());
    }

    public String name() {
        return name;
    }

    @Override
    public Map<String, Object> debug() {
        double rawAngleRad = getRawAngleRad();
        double angleAfterOffsetRad = getAngleAfterOffsetRad();

        Map<String, Object> map = new HashMap<>();
        map.put("servoName", name);
        map.put("rawVoltage", turnEncoder.getVoltage());
        map.put("rawAngleRad", rawAngleRad);
        map.put("rawAngleDeg", Math.toDegrees(rawAngleRad));
        map.put("angleAfterOffsetRad", angleAfterOffsetRad);
        map.put("angleAfterOffsetDeg", Math.toDegrees(angleAfterOffsetRad));
        map.put("continuousEncoderAngleRad", getContinuousEncoderAngleRad());
        map.put("continuousEncoderAngleDeg", Math.toDegrees(getContinuousEncoderAngleRad()));
        map.put("rejectedAngleSpikes", scaledAngleSupplier == null ? 0 : scaledAngleSupplier.getRejectedSpikeCount());
        map.put("servoPower", turnServo.getPower());
        map.put("drivePower", driveMotor.getPower());
        return map;
    }

    public double getContinuousEncoderAngleRad() {
        return scaledAngleSupplier == null ? Double.NaN : scaledAngleSupplier.getContinuousEncoderAngleRad();
    }

    public void setBacklashDeadbandDegrees(double backlashDeadbandDegrees) {
        backlashDeadbandRad = Math.toRadians(backlashDeadbandDegrees);
    }

    public void setMaxEncoderVelocityDegreesPerSecond(double maxEncoderVelocityDegreesPerSecond) {
        if (scaledAngleSupplier != null) {
            scaledAngleSupplier.setMaxEncoderVelocityDegreesPerSecond(maxEncoderVelocityDegreesPerSecond);
        }
    }

    private double wheelErrorToEncoderError(double wheelErrorRad) {
        double encoderErrorRad = wheelErrorRad * wheelToEncoderRatio;
        return encoderReversed.get() ? encoderErrorRad : -encoderErrorRad;
    }

    private double getAnalogMinVoltage() {
        return analogMinVoltage.get();
    }

    private double getAnalogMaxVoltage() {
        return analogMaxVoltage.get();
    }
}
