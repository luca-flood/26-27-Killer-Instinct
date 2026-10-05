package org.firstinspires.ftc.teamcode.pedroPathing.swerve;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PatchedCoaxialPod;

/** Passive diagnostic for the non-1:1 encoder ratio; not an upstream Pedro tuner. */
@TeleOp(name = "Swerve Angle Tracking Test", group = "Pedro Swerve")
public class SwerveAngleTrackingTest extends OpMode {
    private PatchedCoaxialPod[] pods;
    private long previousLoop;

    @Override
    public void init() {
        telemetry.addLine("Align wheels forward before START. Then rotate manually; no powered motion.");
        telemetry.update();
    }

    @Override
    public void start() {
        pods = Constants.createPods(hardwareMap);
        for (PatchedCoaxialPod pod : pods) {
            pod.setMotorPower(0);
            pod.setServoPower(0);
            pod.getAngle();
        }
        previousLoop = System.nanoTime();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        telemetry.addData("loop ms", (now - previousLoop) / 1e6);
        previousLoop = now;
        for (PatchedCoaxialPod pod : pods) {
            telemetry.addData(pod.name(), pod.debug());
        }
        telemetry.addLine("Check 0/90/180/270 and encoder wraps in both directions.");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (pods != null) {
            for (PatchedCoaxialPod pod : pods) {
                pod.setServoPower(0);
                pod.setMotorPower(0);
            }
        }
    }
}
