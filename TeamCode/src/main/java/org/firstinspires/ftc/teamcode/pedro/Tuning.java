package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.ThreeWheelIMULocalizer;
import com.pedropathing.tuning.autotune.*;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelIMUTuner;

// TODO(kim 09/23/26): Run auto tuner for drivetrain
// TODO(kim 09/23/26): Run auto tuner for localization
// TODO(kim 09/23/26): Run auto tuner for foresight
// these are separate todos because they are technically independent

public class Tuning {
    // Drivetrain
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
    // Localization
    @Tuner
    public static Procedure threeWheelIMUTuner() {
        return new ThreeWheelIMUTuner();
    }

    // Foresight
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new ThreeWheelIMULocalizer(hardwareMap, Constants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    // Tests
    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), (hardwareMap -> new ThreeWheelIMULocalizer(hardwareMap, Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
    }
}
