package org.firstinspires.ftc.teamcode.Mechanism;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public final class intakeSubsystem implements Mechanism {
    private final NextMotor intake = new NextMotor("intake");

    public final Command eat = setSuck(0.55);
    public final Command sleep = setSuck(0.0);

    public intakeSubsystem() {
        intake.setDirection(NextMotor.Direction.FORWARD);
    }

    public Command setSuck(double power) {
        return instant(() -> intake.setThrottle(Range.clip(power, -1.0, 1.0)));
    }
}
