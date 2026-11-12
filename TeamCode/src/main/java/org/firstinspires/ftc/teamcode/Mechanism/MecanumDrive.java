package org.firstinspires.ftc.teamcode.Mechanism;

import  com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.control.drive.MecanumKinematics;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.drive.DriveCommands;

public final class MecanumDrive implements Mechanism {
    private static final double DRIVER_POWER = 0.7;
    private static final double SECONDARY_POWER = 0.30;
    private static final float TURN_GAIN = 1.1f;

    private final NextMotor frontLeft = new NextMotor("frontLeft");
    private final NextMotor frontRight = new NextMotor("frontRight");
    private final NextMotor backLeft = new NextMotor("backLeft");
    private final NextMotor backRight = new NextMotor("backRight");
    private final NextMotor[] motors = {frontLeft, frontRight, backLeft, backRight};

    public MecanumDrive() {
        // NextFTC's stick signs are opposite fullTest's. Use the Pedro motor directions.
        frontLeft.setDirection(NextMotor.Direction.FORWARD);
        frontRight.setDirection(NextMotor.Direction.FORWARD);
        backLeft.setDirection(NextMotor.Direction.REVERSE);
        backRight.setDirection(NextMotor.Direction.FORWARD);
    }

    public Command drive(Gamepad driver, Gamepad secondary) {
        Gamepad driveInput = new Gamepad();
        Command driveCommand = DriveCommands.mecanumDrive(
                frontLeft, frontRight, backLeft, backRight, driveInput,
                new MecanumKinematics(1.0));

        return Command.build()
                .requiring(this)
                .setStart(driveCommand::start)
                .setExecute(() -> {
                    double powerScale = selectInput(driveInput, driver, secondary);
                    driveCommand.execute();
                    applyWheelPowers(powerScale);
                })
                .setDone(driveCommand::done)
                .setEnd(end -> {
                    driveCommand.end(end);
                    applyWheelPowers(0.0);
                });
    }

    private static double selectInput(Gamepad driveInput, Gamepad driver, Gamepad secondary) {
        if (hasDriveInput(driver, 0.2f)) {
            copySticks(driver, driveInput);
            return DRIVER_POWER;
        }
        if (hasDriveInput(secondary, 0.1f)) {
            copySticks(secondary, driveInput);
            return SECONDARY_POWER;
        }
        driveInput.left_stick_x = 0.0f;
        driveInput.left_stick_y = 0.0f;
        driveInput.right_stick_x = 0.0f;
        return 0.0;
    }

    private static boolean hasDriveInput(Gamepad gamepad, float rightStickThreshold) {
        return Math.abs(gamepad.left_stick_x) >= 0.1f
                || Math.abs(gamepad.left_stick_y) >= 0.1f
                || Math.abs(gamepad.right_stick_x) >= rightStickThreshold
                || Math.abs(gamepad.right_stick_y) >= rightStickThreshold;
    }

    private static void copySticks(Gamepad source, Gamepad destination) {
        destination.left_stick_x = source.left_stick_x;
        destination.left_stick_y = source.left_stick_y;
        destination.right_stick_x = source.right_stick_x * TURN_GAIN;
    }

    private void applyWheelPowers(double scale) {
        for (NextMotor motor : motors) {
            // Scale the normalized throttle from the NextFTC command.
            NextMotor.ControlType.Throttle output =
                    (NextMotor.ControlType.Throttle) motor.getControlType();
            motor.setThrottle(output.getThrottle() * scale);
            motor.update();
        }
    }
}
