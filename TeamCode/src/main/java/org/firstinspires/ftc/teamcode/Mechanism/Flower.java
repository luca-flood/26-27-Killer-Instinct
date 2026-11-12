package org.firstinspires.ftc.teamcode.Mechanism;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public final class Flower implements Mechanism {
    private final NextServo rightflower = new NextServo("flower1");
    private final NextServo leftflower = new NextServo("flower2");

    public void setPosition(double position) {
        rightflower.setPosition(position);
        leftflower.setPosition(1-position);
    }

    public double getRightPosition() {
        return rightflower.getPosition();
    }

    public double getLeftPosition() {
        return leftflower.getPosition();
    }

    public Command up() {
        return instant(() -> setPosition(.810));
    }
    public Command down() {
        return instant(() -> setPosition(.025));
    }
}
