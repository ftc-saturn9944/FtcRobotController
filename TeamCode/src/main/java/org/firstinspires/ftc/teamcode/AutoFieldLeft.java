package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoFieldLeft extends CommandOpMode{
    private RobotSetupLaunch robot;
    private SequentialCommandGroup driving;
    private ParallelDeadlineGroup auto;
    public void initialize() {
        robot = new RobotSetupLaunch(hardwareMap);
        telemetry.setAutoClear(false);
        telemetry.update();
        telemetry.addLine("startInitialization");
        telemetry.update();
        //Driving
        driving = new SequentialCommandGroup();
        driving.addCommands(
                new DriveSeconds(
                        robot.drive,
                        500,
                        "up",
                        robot.imu,
                        false
                ),
                new TimerCommand(150),
                new DriveSeconds(
                        robot.drive,
                        450,
                        "right",
                        robot.imu,
                        false
                ),
                new TimerCommand(100),
                new RotateDrive(
                        robot.drive,
                        robot.imu,
                        50
                ),
                new TimerCommand(1000),
                robot.triggerRelease,
                new TimerCommand(1500));
        auto = new ParallelDeadlineGroup(
                driving,
                robot.launchRaise
        );
        schedule(auto);


    }
}
