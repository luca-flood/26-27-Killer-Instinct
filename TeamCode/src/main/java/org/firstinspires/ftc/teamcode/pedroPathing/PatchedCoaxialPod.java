package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ftc.drivetrains.SwervePod;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PatchedCoaxialPod implements SwervePod {
    private final AnalogInput turnEncoder;
    private final CRServo turnServo;
    private final DcMotorEx driveMotor;
    private final PIDFController turnPID;
    private final Pose offset;
    private final double angleOffsetRad;
    private final String servoLabel;
    private final double analogMinVoltage;
    private final double analogMaxVoltage;
    private final boolean encoderReversed;
    private final double wheelToEncoderRatio;
    private final double encoderToWheelRatio;
    private final ElapsedTime sampleTimer = new ElapsedTime();

    private double motorCachingThreshold = 0.01;
    private double servoCachingThreshold = 0.01;
    private double backlashDeadbandRad = Math.toRadians(2.0);
    private double maxEncoderVelocityRadPerSecond = Math.toRadians(1440.0);
    private double lastWrappedEncoderAngleRad = 0.0;
    private double continuousEncoderAngleRad = 0.0;
    private boolean hasEncoderSample = false;
    private double lastDrivePower = 0.0;
    private double lastTurnPower = 0.0;

    public PatchedCoaxialPod(
            HardwareMap hardwareMap,
            String motorName,
            String servoName,
            String turnEncoderName,
            PIDFCoefficients turnPIDFCoefficients,
            DcMotorSimple.Direction driveDirection,
            CRServo.Direction servoDirection,
            double angleOffsetRad,
            Pose podOffset,
            double analogMinVoltage,
            double analogMaxVoltage,
            boolean encoderReversed,
            double wheelDegreesPerEncoderRevolution) {
        driveMotor = hardwareMap.get(DcMotorEx.class, motorName);
        turnServo = hardwareMap.get(CRServo.class, servoName);
        turnEncoder = hardwareMap.get(AnalogInput.class, turnEncoderName);
        turnPID = new PIDFController(turnPIDFCoefficients);
        this.angleOffsetRad = angleOffsetRad;
        this.offset = podOffset;
        this.analogMinVoltage = analogMinVoltage;
        this.analogMaxVoltage = analogMaxVoltage;
        this.encoderReversed = encoderReversed;
        this.encoderToWheelRatio = wheelDegreesPerEncoderRevolution / 360.0;
        this.wheelToEncoderRatio = 360.0 / wheelDegreesPerEncoderRevolution;
        this.servoLabel = servoName;

        driveMotor.setDirection(driveDirection);
        turnServo.setDirection(servoDirection);
        setMotorToFloat();
        turnServo.setPower(0.0);
    }

    @Override
    public Pose getOffset() {
        return offset;
    }

    @Override
    public double getAngle() {
        return getWheelAngleAfterOffsetRad();
    }

    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        return MathFunctions.normalizeAngle(wheelTheta);
    }

    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        double actualWheelRad = getAngle();
        double desiredWheelRad = adjustThetaForEncoder(targetAngleRad);

        double wheelErrorRad = shortestSignedError(actualWheelRad, desiredWheelRad);

        if (Math.abs(wheelErrorRad) > (Math.PI / 2.0)) {
            desiredWheelRad = MathFunctions.normalizeAngle(desiredWheelRad + Math.PI);
            drivePower = -drivePower;
            wheelErrorRad = shortestSignedError(actualWheelRad, desiredWheelRad);
        }

        if (Math.abs(wheelErrorRad) < backlashDeadbandRad) {
            turnPID.updateFeedForwardInput(0.0);
            setTurnPower(0.0, ignoreAngleChanges);
        } else {
            double encoderErrorRad = wheelErrorToEncoderError(wheelErrorRad);
            turnPID.updateFeedForwardInput(Math.signum(encoderErrorRad));
            turnPID.updateError(encoderErrorRad);
            setTurnPower(MathFunctions.clamp(turnPID.run(), -1.0, 1.0), ignoreAngleChanges);
        }

        setDrivePower(drivePower);
    }

    @Override
    public void setToFloat() {
        setMotorToFloat();
    }

    @Override
    public void setToBreak() {
        setMotorToBrake();
    }

    public void setMotorToFloat() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setMotorToBrake() {
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getRawAngleRad() {
        double range = analogMaxVoltage - analogMinVoltage;
        if (range == 0.0) {
            return 0.0;
        }

        double normalized = (turnEncoder.getVoltage() - analogMinVoltage) / range;
        return MathFunctions.clamp(normalized, 0.0, 1.0) * (2.0 * Math.PI);
    }

    public double getWheelAngleAfterOffsetRad() {
        double encoderDelta = getContinuousEncoderAngleRad() - angleOffsetRad;
        if (!encoderReversed) {
            encoderDelta *= -1.0;
        }
        return MathFunctions.normalizeAngle(encoderDelta * encoderToWheelRatio);
    }

    public double getContinuousEncoderAngleRad() {
        updateContinuousEncoderAngle();
        return continuousEncoderAngleRad;
    }

    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public void setServoCachingThreshold(double servoCachingThreshold) {
        this.servoCachingThreshold = servoCachingThreshold;
    }

    public void setBacklashDeadbandDegrees(double backlashDeadbandDegrees) {
        this.backlashDeadbandRad = Math.toRadians(backlashDeadbandDegrees);
    }

    public void setMaxEncoderVelocityDegreesPerSecond(double maxEncoderVelocityDegreesPerSecond) {
        this.maxEncoderVelocityRadPerSecond = Math.toRadians(maxEncoderVelocityDegreesPerSecond);
    }

    @Override
    public String debugString() {
        double rawAngleRad = getRawAngleRad();
        double continuousAngleRad = getContinuousEncoderAngleRad();
        double wheelAngleRad = getWheelAngleAfterOffsetRad();
        return servoLabel + " {"
                + "\nraw encoder angle (rad/deg) = " + rawAngleRad + " / " + Math.toDegrees(rawAngleRad)
                + "\ncontinuous encoder angle (rad/deg) = " + continuousAngleRad + " / " + Math.toDegrees(continuousAngleRad)
                + "\nwheel angle after offset (rad/deg) = " + wheelAngleRad + " / " + Math.toDegrees(wheelAngleRad)
                + "\nservo Power = " + turnServo.getPower()
                + "\ndrive Power = " + driveMotor.getPower()
                + "\n}";
    }

    private void updateContinuousEncoderAngle() {
        double wrappedAngleRad = getRawAngleRad();

        if (!hasEncoderSample) {
            lastWrappedEncoderAngleRad = wrappedAngleRad;
            continuousEncoderAngleRad = wrappedAngleRad;
            hasEncoderSample = true;
            sampleTimer.reset();
            return;
        }

        double deltaRad = normalizeSignedAngle(wrappedAngleRad - lastWrappedEncoderAngleRad);
        double elapsedSeconds = Math.max(sampleTimer.seconds(), 0.001);
        double maxDeltaRad = maxEncoderVelocityRadPerSecond * elapsedSeconds;

        if (Math.abs(deltaRad) <= maxDeltaRad) {
            continuousEncoderAngleRad += deltaRad;
            lastWrappedEncoderAngleRad = wrappedAngleRad;
        }

        sampleTimer.reset();
    }

    private double wheelErrorToEncoderError(double wheelErrorRad) {
        double encoderErrorRad = wheelErrorRad * wheelToEncoderRatio;
        return encoderReversed ? encoderErrorRad : -encoderErrorRad;
    }

    private double normalizeSignedAngle(double angleRad) {
        while (angleRad > Math.PI) {
            angleRad -= 2.0 * Math.PI;
        }
        while (angleRad <= -Math.PI) {
            angleRad += 2.0 * Math.PI;
        }
        return angleRad;
    }

    private double shortestSignedError(double actualRad, double desiredRad) {
        double magnitude = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
        double direction = MathFunctions.getTurnDirection(actualRad, desiredRad);
        return (magnitude == Math.PI) ? -Math.PI : magnitude * direction;
    }

    private void setTurnPower(double turnPower, boolean ignoreAngleChanges) {
        if (ignoreAngleChanges) {
            turnPower = 0.0;
        }

        if (Math.abs(turnPower - lastTurnPower) > servoCachingThreshold || (turnPower == 0.0 && lastTurnPower != 0.0)) {
            lastTurnPower = turnPower;
            turnServo.setPower(turnPower);
        }
    }

    private void setDrivePower(double drivePower) {
        if (Math.abs(drivePower - lastDrivePower) > motorCachingThreshold || (drivePower == 0.0 && lastDrivePower != 0.0)) {
            lastDrivePower = drivePower;
            driveMotor.setPower(drivePower);
        }
    }
}
