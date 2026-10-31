package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Follower create(HardwareMap h) {
        return org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(h);
    }
}
