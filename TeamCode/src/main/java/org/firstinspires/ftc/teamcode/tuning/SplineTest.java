package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, Math.PI/2);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();


            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        // [X] Move to score pre-load
                        .strafeToLinearHeading(new Vector2d(10, 10), Math.PI*.75)
                        .strafeToConstantHeading(new Vector2d(5,15))
                        .waitSeconds(5.0)

                         // [X] Move to collect first piece
                         .strafeToConstantHeading(new Vector2d( 7, 13))
                         .strafeToLinearHeading(new Vector2d(12,14), 0)
                         .waitSeconds(5.0)

                         // [X] Return to scoring - 1
                        .strafeToLinearHeading(new Vector2d(5, 15), Math.PI*.75)
                        .waitSeconds(5.0)

                         // [X] Collect second piece
                         .strafeToConstantHeading(new Vector2d( 7, 13))
                        .strafeToLinearHeading(new Vector2d(12, 23), Math.PI/10.0)
                        .waitSeconds(5.0)

                         // [X] Return to scoring - 2
                         // .strafeToConstantHeading(new Vector2d(7, 13))
                         .strafeToLinearHeading(new Vector2d(5, 15), Math.PI*.75)
                        .waitSeconds(5.0)

                         // [ ] Move to park
                         .strafeToConstantHeading(new Vector2d( 7, 13))
                         .strafeToLinearHeading(new Vector2d(36,14), Math.PI/10)
                        .waitSeconds(0.1)
                         .splineTo(new Vector2d(60,-12), -Math.PI/3.0)

                       .build());
        } /*else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
          */
         else {
            throw new RuntimeException();
        }
    }
}
