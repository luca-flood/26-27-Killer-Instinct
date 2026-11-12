package org.firstinspires.ftc.teamcode.Mechanism;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public final class Blocker implements Mechanism {
    private final NextServo servo = new NextServo("blocker");

    public Command block() {
        return instant(() -> servo.setPosition(1.0));
    }

    public Command allowFeed() {
        return instant(() -> servo.setPosition(0.5));
    }
}
