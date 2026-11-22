package org.firstinspires.ftc.teamcode.Mechanism;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

import dev.nextftc.robot.Mechanism;

/** Reads the pose and HIVE state from the active BluePower flat map. */
public final class Limelight implements Mechanism, AutoCloseable {
    private static final double HALF_FIELD_IN = 72.0;
    private static final long MAX_FRAME_AGE_MS = 250;
    private static final long MAX_FRAME_AGE_NS = TimeUnit.MILLISECONDS.toNanos(MAX_FRAME_AGE_MS);
    private static final double MIN_SETTLED_ANGLE_RAD = Math.toRadians(23);
    private static final int HIVE_CONFIRM_FRAMES = 3;
    private static final int RED_TAGS = 1;
    private static final int BLUE_TAGS = 2;
    private static final int OTHER_TAGS = 4;
    private static final long PIPELINE_RETRY_MS = 500;

    public enum TipStatus {
        UNKNOWN("Unknown"), MOVING("Moving / between states"),
        NOT_TIPPED("Not tipped"), TIPPED("Tipped");

        private final String label;

        TipStatus(String label) {
            this.label = label;
        }

        @Override
        public String toString() {
            return label;
        }
    }

    public static final class PoseReading {
        /** Raw MegaTag1 X/Y in inches from field center, or null when unavailable. */
        public final Vector2D rawXYIn;
        /** Corrected Pedro pose in inches and radians, or null when unavailable. */
        public final Pose pose;
        public final List<Integer> tagIds;
        public final String status;

        private PoseReading(Vector2D rawXYIn, Pose pose, List<Integer> tagIds, String status) {
            this.rawXYIn = rawXYIn;
            this.pose = pose;
            this.tagIds = Collections.unmodifiableList(tagIds);
            this.status = status;
        }

        /** Uncorrected MegaTag1 X/Y in Pedro field coordinates, or null when unavailable. */
        public Vector2D rawPedroXYIn() {
            return rawXYIn == null ? null : toPedroXY(rawXYIn.x(), rawXYIn.y());
        }
    }

    private Limelight3A camera;
    private int pipelineIndex;
    private Thread pipelineSwitchThread;
    private double cameraTimestamp = Double.NaN;
    private long frameSeenAt;
    private PoseReading reading = new PoseReading(null, null, Collections.emptyList(), "Limelight unavailable");
    private TipStatus tipCandidate = TipStatus.UNKNOWN;
    private int agreeingFrames;

    /** Looks up the camera and requests its pipeline during INIT. */
    public void configure(HardwareMap hardwareMap, int pipelineIndex) {
        close();
        Limelight3A device = hardwareMap.tryGet(Limelight3A.class, "limelight");
        if (device == null) {
            return;
        }
        camera = device;
        this.pipelineIndex = pipelineIndex;

        // A pipeline switch waits for an HTTP response. Keep it off the OpMode thread.
        pipelineSwitchThread = new Thread(() -> {
            try {
                while (!Thread.currentThread().isInterrupted() && !device.pipelineSwitch(pipelineIndex)) {
                    Thread.sleep(PIPELINE_RETRY_MS);
                }
            } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
            } catch (RuntimeException exception) {
                RobotLog.ee("Limelight", exception, "Could not select pipeline %d", pipelineIndex);
            }
        }, "LimelightPipeline");
        pipelineSwitchThread.setDaemon(true);
        pipelineSwitchThread.start();
        rejectPose(Collections.emptyList(), "Waiting for pipeline " + pipelineIndex);
    }

    /** Starts polling on PLAY. */
    public void start() {
        if (camera != null) {
            camera.setPollRateHz(50);
            camera.start();
        }
    }

    /** Reports whether the observed HIVE is opposite its official reset position. */
    public Command checkTip(Consumer<TipStatus> onResult) {
        return Commands.instant(() -> onResult.accept(tipStatus()));
    }

    /** Returns the latest raw MegaTag1 X/Y, corrected pose when available, and visible tag IDs. */
    public Command readPose(Consumer<PoseReading> onResult) {
        return Commands.instant(() -> onResult.accept(reading));
    }

    @Override
    public void periodic() {
        if (camera == null || !camera.isRunning()) {
            return;
        }
        LLResult frame = camera.getLatestResult();
        double timestamp = frame.getTimestamp();
        long now = System.nanoTime();
        long receiveAge = frame.getStaleness();
        if (!Double.isFinite(timestamp) || timestamp <= 0 || receiveAge < 0 || receiveAge > MAX_FRAME_AGE_MS) {
            rejectPose(Collections.emptyList(), "Waiting for a fresh frame");
            return;
        }
        if (frame.getPipelineIndex() != pipelineIndex) {
            rejectPose(Collections.emptyList(), "Waiting for pipeline " + pipelineIndex);
            return;
        }

        boolean frameGap = now - frameSeenAt > MAX_FRAME_AGE_NS;
        if (timestamp == cameraTimestamp) {
            if (frameGap) {
                rejectPose(Collections.emptyList(), "Waiting for a fresh frame");
            }
            return;
        }
        if (timestamp < cameraTimestamp || frameGap) {
            updateTip(TipStatus.UNKNOWN);
        }
        cameraTimestamp = timestamp;
        frameSeenAt = now;
        readFrame(frame);
    }

    private void readFrame(LLResult frame) {
        List<Integer> tagIds = new ArrayList<>();
        int tagGroups = 0;
        for (FiducialResult tag : frame.getFiducialResults()) {
            int id = tag.getFiducialId();
            tagIds.add(id);
            tagGroups |= tagGroup(id);
        }
        if (!frame.isValid() || frame.getBotposeTagCount() <= 0) {
            rejectPose(tagIds, "No mapped tags");
            return;
        }
        Pose3D raw = frame.getBotpose();
        Position inches = raw.getPosition().toUnit(DistanceUnit.INCH);
        double yaw = raw.getOrientation().getYaw(AngleUnit.RADIANS);
        double pitch = raw.getOrientation().getPitch(AngleUnit.RADIANS);
        double roll = raw.getOrientation().getRoll(AngleUnit.RADIANS);
        if (inches.x == 0 && inches.y == 0 && inches.z == 0 && yaw == 0 && pitch == 0 && roll == 0) {
            rejectPose(tagIds, "No MT1 estimate");
            return;
        }
        if (!Double.isFinite(inches.x) || !Double.isFinite(inches.y)) {
            rejectPose(tagIds, "Invalid MT1 X/Y");
            return;
        }

        Vector2D rawXYIn = Vector2D.cartesian(inches.x, inches.y);
        if (tagGroups != RED_TAGS && tagGroups != BLUE_TAGS) {
            rejectCorrection(rawXYIn, tagIds, "Need tags from one HIVE");
            return;
        }

        HiveHingeSolver.Result result = HiveHingeSolver.solve(inches.x, inches.y, inches.z, yaw, pitch);
        if (!result.isValid()) {
            rejectCorrection(rawXYIn, tagIds, result.status.name());
            return;
        }
        Vector2D correctedXYIn = toPedroXY(result.correctedXIn, result.correctedYIn);
        Pose pose = new Pose(correctedXYIn.x(), correctedXYIn.y(), result.correctedYawRad - Math.PI / 2.0);
        reading = new PoseReading(rawXYIn, pose, tagIds, "OK");

        // Positive angle lowers the audience cell. At reset that cell is up only on red.
        if (Math.abs(result.hiveAngleRad) <= MIN_SETTLED_ANGLE_RAD) {
            updateTip(TipStatus.MOVING);
        } else {
            boolean audienceUp = result.hiveAngleRad < 0;
            boolean startsAudienceUp = tagGroups == RED_TAGS;
            updateTip(audienceUp == startsAudienceUp ? TipStatus.NOT_TIPPED : TipStatus.TIPPED);
        }
    }

    private static Vector2D toPedroXY(double ftcXIn, double ftcYIn) {
        return Vector2D.cartesian(ftcYIn + HALF_FIELD_IN, HALF_FIELD_IN - ftcXIn);
    }

    private void rejectPose(List<Integer> tagIds, String status) {
        rejectCorrection(null, tagIds, status);
    }

    private void rejectCorrection(Vector2D rawXYIn, List<Integer> tagIds, String status) {
        reading = new PoseReading(rawXYIn, null, tagIds, status);
        updateTip(TipStatus.UNKNOWN);
    }

    private void updateTip(TipStatus candidate) {
        if (candidate == TipStatus.UNKNOWN || candidate == TipStatus.MOVING) {
            agreeingFrames = 0;
        } else if (candidate == tipCandidate) {
            agreeingFrames = Math.min(agreeingFrames + 1, HIVE_CONFIRM_FRAMES);
        } else {
            agreeingFrames = 1;
        }
        tipCandidate = candidate;
    }

    private TipStatus tipStatus() {
        return tipCandidate == TipStatus.MOVING || agreeingFrames >= HIVE_CONFIRM_FRAMES
                ? tipCandidate : TipStatus.UNKNOWN;
    }

    private static int tagGroup(int id) {
        if (id >= 30 && id <= 37) return RED_TAGS;
        if (id >= 38 && id <= 45) return BLUE_TAGS;
        return OTHER_TAGS;
    }

    @Override
    public void close() {
        if (pipelineSwitchThread != null) {
            pipelineSwitchThread.interrupt();
            pipelineSwitchThread = null;
        }
        if (camera != null) {
            camera.stop();
            camera = null;
        }
        cameraTimestamp = Double.NaN;
        frameSeenAt = 0;
        rejectPose(Collections.emptyList(), "Limelight unavailable");
    }
}
