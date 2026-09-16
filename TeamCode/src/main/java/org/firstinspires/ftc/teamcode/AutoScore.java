package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Timer;

@Autonomous
public class AutoScore extends CommandOpMode {
    private RobotSetupLaunch robot;
    private SequentialCommandGroup driving;
    private ParallelDeadlineGroup auto;
    public void initialize(){
        robot = new RobotSetupLaunch(hardwareMap);
        telemetry.setAutoClear(false);
        telemetry.update();
        telemetry.addLine("startInitialization");
        telemetry.update();
        // Driving
        driving = new SequentialCommandGroup();
        driving.addCommands(
                new TimerCommand(20),
                robot.triggerRelease,
                new DriveSeconds(
                        robot.drive,
                        600,
                        "up",
                        robot.imu,
                        false),
                new DriveSeconds(
                        robot.drive,
                        600,
                        "left",
                        robot.imu,
                        false));
        schedule(driving);

    }
}
