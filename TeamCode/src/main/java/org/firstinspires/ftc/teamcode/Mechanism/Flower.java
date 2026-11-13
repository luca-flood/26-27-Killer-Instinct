package org.firstinspires.ftc.teamcode.Mechanism;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

@Disabled
public final class Flower implements Mechanism {
    // A negative tolerance lets repeated targets reach the FTC servo controller.
    private final NextServo rightflower = new NextServo("flower1", -1.0);
    private final NextServo leftflower = new NextServo("flower2", -1.0);

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
        return instant(() -> {
            setPosition(0.042);
            rightflower.enable();
            leftflower.enable();
        });
    }
}
