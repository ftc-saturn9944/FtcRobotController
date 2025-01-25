package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous
public class RRAutoTest extends LinearOpMode {
    private int rotateScore = 11600;
    private int rotatePrep = 8000;
    private int rotateIntake = 942;
    private int rotatePark;
    private int liftScore = 4350;
    private int liftPark;
    private double gripperTime = 1;
    private MecanumDrive drive;

    private Pose2d beginPose = new Pose2d(0,0, Math.PI/2);
    @Override
    public void runOpMode() {
        ActionLift lift = new ActionLift(hardwareMap);
        ActionRotate rotate = new ActionRotate(hardwareMap);
        ActionGripper gripper = new ActionGripper(hardwareMap);
        ActionWrist wrist = new ActionWrist(hardwareMap, 0.44, 0.1);
        Actions.runBlocking(wrist.center());

        drive = new MecanumDrive(hardwareMap, beginPose);
        TrajectoryActionBuilder preloadScore = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(10, 10), Math.PI*.75)
                .strafeToConstantHeading(new Vector2d(5,15));
        Action runPreloadScore = preloadScore.build();
        TrajectoryActionBuilder firstIntake = preloadScore.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d( 7, 13))
                .strafeToLinearHeading(new Vector2d(12,14), 0);
        Action runFirstIntake = firstIntake.build();
        TrajectoryActionBuilder firstScore = firstIntake.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(5, 15), Math.PI*.75);
        Action runFirstScore = firstScore.build();
        TrajectoryActionBuilder secondIntake = firstScore.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d( 7, 13))
                .strafeToLinearHeading(new Vector2d(12, 23), Math.PI/10.0);
        Action runSecondIntake = secondIntake.build();
        TrajectoryActionBuilder secondScore = secondIntake.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(6, 14), Math.PI*.75);
        Action runSecondScore = secondScore.build();


        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        rotate.increaseHeight(rotatePrep),
                        new ParallelAction(
                                rotate.increaseHeight(rotateScore),
                                lift.increaseHeight(liftScore)
                        ),
                        runPreloadScore,
                        gripper.release(gripperTime),
                        //move to 1st piece
                        runFirstIntake,
                        rotate.decreaseHeight(rotateIntake),
                        gripper.intake(gripperTime),
                       rotate.increaseHeight(rotateScore),
                        //move to score
                        runFirstScore,
                        gripper.release(gripperTime),
                        //move to 2nd piece
                        runSecondIntake,
//                        rotate.decreaseHeight(rotateIntake),
                        gripper.intake(gripperTime),
//                        rotate.increaseHeight(rotateScore),
                        //move to score
                        runSecondScore,
                        gripper.release(gripperTime),
                        //move to park
                        new SleepAction(1)
                )
        );
    }
}
