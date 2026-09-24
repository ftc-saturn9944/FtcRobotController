package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.ThreeWheelIMULocalizer;
import com.pedropathing.tuning.autotune.*;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelIMUTuner;

// TODO(kim 09/23/26): Run auto tuner for drivetrain (impossible)
// TODO(kim 09/23/26): Run auto tuner for localization (maybe possible idk)
//  (i also dont know what the encoder names are pls figure that out)
// TODO(kim 09/23/26): Run auto tuner for foresight (impossible)
// these are separate todos because they are technically independent

public class Tuning {
    // Drivetrain
    // FIXME(kim 09/23/26): add drivetrain, i dont know what mecanum or swerve means or which we use
    //  everything after may be impossible without this done.
    //  https://pedropathing.com/docs/pathing/tuning/drivetrain/mecanum

    // Localization
    @Tuner
    public static Procedure threeWheelIMUTuner() {
        return new ThreeWheelIMUTuner();
    }

    // Foresight
    // TODO(kim 09/23/26): the "Mecanum" hardwareMap needs to be updated if we use Swerve, leave it otherwise.
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new ThreeWheelIMULocalizer(hardwareMap, Constants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    // Tests
    // TODO(kim 09/23/26): same as foresight, update hardwareMap if we use Swerve.
    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), (hardwareMap -> new ThreeWheelIMULocalizer(hardwareMap, Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
    }
}
