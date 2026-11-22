package org.firstinspires.ftc.teamcode.Mechanism;

/** Converts a MegaTag1 pose from one flat HIVE reference map into the field frame. */
final class HiveHingeSolver {
    // Geometry and acceptance limits from BluePower's derivation:
    // https://github.com/BluePowerRobotics/Team19725_BIOBUZZ/blob/6b788b4ba7fff43350b1a4c0c4ee96eb09ea0fdd/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/OneMap_MT1Localizer.md
    private static final double HINGE_HEIGHT_IN = 43.95;
    private static final double MAX_TILT_RAD = Math.toRadians(35.0);
    private static final double MAX_PITCH_ERROR_RAD = Math.toRadians(10.0);
    private static final double EPSILON = 1e-12;

    enum Status {
        SOLVED,
        NON_FINITE_POSE,
        NO_FLOOR_INTERSECTION,
        TILT_OUT_OF_RANGE,
        PITCH_MISMATCH,
        UNDEFINED_HEADING
    }

    static final class Result {
        final Status status;
        final double hiveAngleRad;
        final double correctedXIn;
        final double correctedYIn;
        final double correctedYawRad;

        private Result(Status status, double hiveAngleRad, double correctedXIn,
                       double correctedYIn, double correctedYawRad) {
            this.status = status;
            this.hiveAngleRad = hiveAngleRad;
            this.correctedXIn = correctedXIn;
            this.correctedYIn = correctedYIn;
            this.correctedYawRad = correctedYawRad;
        }

        boolean isValid() {
            return status == Status.SOLVED;
        }
    }

    private HiveHingeSolver() {}

    /** Raw positions are inches from field center, with tile height at Z=0. Angles are radians. */
    static Result solve(double rawXIn, double rawYIn, double rawZIn,
                        double rawYawRad, double rawPitchRad) {
        if (!Double.isFinite(rawXIn) || !Double.isFinite(rawYIn) || !Double.isFinite(rawZIn)
                || !Double.isFinite(rawYawRad) || !Double.isFinite(rawPitchRad)) {
            return rejected(Status.NON_FINITE_POSE);
        }

        // Solve x' sin(phi) + (z' - h) cos(phi) = -h using phase and radius.
        double hingeRelativeZ = rawZIn - HINGE_HEIGHT_IN;
        double radius = Math.hypot(rawXIn, hingeRelativeZ);
        if (!Double.isFinite(radius)) {
            return rejected(Status.NON_FINITE_POSE);
        }
        if (radius + EPSILON < HINGE_HEIGHT_IN) {
            return rejected(Status.NO_FLOOR_INTERSECTION);
        }

        double phase = Math.atan2(hingeRelativeZ, rawXIn);
        double principal = Math.asin(Math.max(-1.0, -HINGE_HEIGHT_IN / radius));
        double root = wrap(principal - phase);
        double otherRoot = wrap(Math.PI - principal - phase);
        // If both roots fit, use the smaller rotation. If neither fits, reject the frame.
        double phi = Math.abs(root) <= Math.abs(otherRoot) ? root : otherRoot;
        if (Math.abs(phi) > MAX_TILT_RAD + EPSILON) {
            return rejected(Status.TILT_OUT_OF_RANGE);
        }

        double cosPhi = Math.cos(phi);
        double sinPhi = Math.sin(phi);
        double cosRawYaw = Math.cos(rawYawRad);
        double expectedPitch = Math.asin(sinPhi * cosRawYaw);
        double pitchError = Math.abs(wrap(rawPitchRad - expectedPitch));
        if (pitchError > MAX_PITCH_ERROR_RAD) {
            return rejected(Status.PITCH_MISMATCH);
        }

        double correctedX = cosPhi * rawXIn - sinPhi * hingeRelativeZ;
        if (!Double.isFinite(correctedX)) {
            return rejected(Status.NON_FINITE_POSE);
        }
        // Rotate the robot's forward vector by -phi before extracting its planar heading.
        double cosRawPitch = Math.cos(rawPitchRad);
        double forwardX = cosPhi * cosRawYaw * cosRawPitch + sinPhi * Math.sin(rawPitchRad);
        double forwardY = Math.sin(rawYawRad) * cosRawPitch;
        if (Math.hypot(forwardX, forwardY) < EPSILON) {
            return rejected(Status.UNDEFINED_HEADING);
        }

        return new Result(Status.SOLVED, -phi, correctedX, rawYIn,
                Math.atan2(forwardY, forwardX));
    }

    private static Result rejected(Status status) {
        return new Result(status, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
    }

    private static double wrap(double angle) {
        double wrapped = angle % (2.0 * Math.PI);
        if (wrapped >= Math.PI) {
            return wrapped - 2.0 * Math.PI;
        }
        return wrapped < -Math.PI ? wrapped + 2.0 * Math.PI : wrapped;
    }
}
