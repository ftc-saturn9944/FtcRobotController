package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.ThreeWheelIMUConfig;
import com.pedropathing.revhub.localizers.ThreeWheelIMULocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    // FIXME(kim 09/23/26): These need values which we get from running their respective auto tuners, they need to be added manually. look at the pedro docs for each one to see how to do that.
    //  https://pedropathing.com/docs/pathing/tuning
    public static MecanumConfig drivetrainConfig;
    public static ThreeWheelIMUConfig localizerConfig;
    public static ForesightConfig foresightConfig;

    public static Follower create(HardwareMap h) {
        return new Follower(
                new ThreeWheelIMULocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}