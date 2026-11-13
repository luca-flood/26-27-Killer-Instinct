package org.firstinspires.ftc.teamcode;

/** Tuned and hardware-specific constants, grouped by subsystem. */
public final class RobotConstants {
    private RobotConstants() {}

    public static final class ShooterConstants {
        private ShooterConstants() {}

        // TODO: set to your motor's encoder counts per output revolution
        public static final double TICKS_PER_REV = 28.0;
        // TODO: how close (in RPM) counts as "at speed"
        public static final double RPM_TOLERANCE = 50.0;

        // Tuned velocity gains; copy results from Shooter Subsystem Test
        public static final double KP = 0.00042;
        public static final double KI = 0.0;
        public static final double KD = 0.0001;
        public static final double KS = 0.125;
        public static final double KV = 0.00029 * 0.6;
        public static final double KA = 0.0;
    }
}
