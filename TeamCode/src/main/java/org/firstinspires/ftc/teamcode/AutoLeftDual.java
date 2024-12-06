package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous(name="Auto Left Dual")
public class AutoLeftDual extends CommandOpMode {

    private RobotSetupDual robot;
    private String parkDir = "right";
    private long delay = 100;
    private SequentialCommandGroup driving;
    private MotorSubsystem liftPower, lowerPower, stopPower;


    public void initialize() {
        robot = new RobotSetupDual(
                "blue",
                hardwareMap
        );
        telemetry.setAutoClear(false);
        telemetry.update();
        telemetry.addLine("startInitialization");
        telemetry.update();

        // Driving
        driving = new SequentialCommandGroup();
        driving.addCommands(
                new ParallelRaceGroup(
                        new TimerCommand(500),
                        robot.mwristRaise
                ),
                new DriveSeconds(
                        robot.drive,
                        10,
                        "left",
                        robot.imu,
                        false
                ),
                new DriveSeconds(
                        robot.drive,
                        0,
                        "stop",
                        robot.imu,
                        false
                ),
                new DriveSeconds(
                        robot.drive,
                        1000,
                        "forward",
                        robot.imu,
                        false
                ),
                new DriveSeconds(
                        robot.drive,
                        0,
                        "stop",
                        robot.imu,
                        false
                ),
                new ParallelRaceGroup(
                        new TimerCommand(2000),
                        robot.mwristRaise,
                        robot.armRaise

                ),
                new ParallelRaceGroup(
                        new TimerCommand(500),
                        robot.intakeBackward
                ),
                new ParallelRaceGroup(
                        new DriveSeconds(
                                robot.drive,
                                1000,
                                "backward",
                                robot.imu,
                                false

                        ),

                        new TimerCommand(2000),
                        robot.mwristLower,
                        robot.armLower
                ),
                new DriveSeconds(
                        robot.drive,
                        0,
                        "stop",
                        robot.imu,
                        false
                ),

                new DriveSeconds(
                        robot.drive,
                        250,
                        "right",
                        robot.imu,
                        false
                ),
                new DriveSeconds(
                        robot.drive,
                        0,
                        "stop",
                        robot.imu,
                        false
                ),
                new DriveSeconds(
                        robot.drive,
                        1000,
                        "forward",
                        robot.imu,
                        false
                ),
                new DriveSeconds(
                        robot.drive,
                        0,
                        "stop",
                        robot.imu,
                        false
                ),
                new ParallelRaceGroup(
                        new TimerCommand(300),
                        robot.intakeForward
                ),
                new DriveSeconds(
                        robot.drive,
                        250,
                        "left",
                        robot.imu,
                        false
                ),
                new DriveSeconds(
                        robot.drive,
                        1000,
                        "forward",
                        robot.imu,
                        false
                ),
                new ParallelRaceGroup(
                      new TimerCommand(500),
                        robot.armRaise,
                        robot.mwristRaise
                ),
                new ParallelRaceGroup(
                        new TimerCommand(250),
                        robot.intakeBackward
                ),
                new ParallelRaceGroup(
                        new TimerCommand(1000),
                        robot.armLower,
                        robot.mwristLower
                )

        );
        schedule(driving);
        /*
          turn,
          High basket,
          grab piece,
          high basket,
          park
         */
    }



     {


    }

}
