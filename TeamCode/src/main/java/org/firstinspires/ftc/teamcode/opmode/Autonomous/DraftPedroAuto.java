package org.firstinspires.ftc.teamcode.opmode.Autonomous;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.control.Command;
import org.firstinspires.ftc.teamcode.control.FollowPathCommand;
import org.firstinspires.ftc.teamcode.control.InstantCommand;
import org.firstinspires.ftc.teamcode.control.SequentialCommand;
import org.firstinspires.ftc.teamcode.control.SubsystemManager;
import org.firstinspires.ftc.teamcode.subsystem.SwerveDriveSubsystem;

import static com.pedropathing.api.Paths.line;

@Disabled
@Autonomous(name = "Draft Pedro Auto", group = "Drafts")
public class DraftPedroAuto extends OpMode {
    private final Pose startPose = new Pose(0.0, 0.0, 0.0);
    private SwerveDriveSubsystem drive;
    private SubsystemManager subsystems;
    private Command routine;

    @Override
    public void init() {
        drive = new SwerveDriveSubsystem(hardwareMap, startPose);
        subsystems = new SubsystemManager(drive);
        subsystems.init();

        Path leaveStart = line(startPose, new Pose(24.0, 0.0, 0.0))
                .linear(startPose, new Pose(24.0, 0.0, 0.0));

        routine = new SequentialCommand(
                new InstantCommand(() -> telemetry.addLine("Starting draft Pedro auto")),
                new FollowPathCommand(drive.getFollower(), leaveStart, true, 0.45),
                new InstantCommand(() -> telemetry.addLine("Draft auto complete"))
        );
    }

    @Override
    public void start() {
        subsystems.start();
        routine.start();
    }

    @Override
    public void loop() {
        routine.update();
        subsystems.update();

        telemetry.addData("auto finished", routine.isFinished());
        telemetry.addData("drive busy", drive.isBusy());
        telemetry.addData("pose", "%s", drive.getPose());
        telemetry.update();

        if (routine.isFinished()) {
            requestOpModeStop();
        }
    }

    @Override
    public void stop() {
        if (routine != null) {
            routine.stop(true);
        }
        if (subsystems != null) {
            subsystems.stop();
        }
    }
}
