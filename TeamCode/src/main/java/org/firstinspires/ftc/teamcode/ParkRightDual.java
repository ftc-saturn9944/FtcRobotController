package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Park Right Dual")
public class ParkRightDual extends CommandOpMode {

    private RobotSetupDual robot;
    private String parkDir = "right";
    private long parkDuration = 1000;
    private long delay = 100;
    private SequentialCommandGroup driving;

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
                new DriveSeconds(
                        robot.drive,
                        0,
                        "stop",
                        robot.imu,
                        false
                ),
                new DriveSeconds(
                        robot.drive,
                        parkDuration,
                        parkDir,
                        robot.imu,
                        false
                ),
                new DriveSeconds(
                        robot.drive,
                        0,
                        "stop",
                        robot.imu,
                        false
                )
        );
        schedule(driving);
    }
    @Override
    public void run(){
        telemetry.clearAll();

        telemetry.update();
        super.run();

    }
}
