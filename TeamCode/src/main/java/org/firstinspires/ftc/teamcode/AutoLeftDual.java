package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous(name="Auto Left Dual", preselectTeleOp = "SystemsTestDualGripper")
public class AutoLeftDual extends CommandOpMode {

    private RobotSetupDual robot;
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

        robot.mwrist.resetEncoder();
        robot.arm.resetEncoder();
        robot.wrist.setPosition(.1);
        robot.mwrist.setDefaultCommand(robot.mwristStop);
        robot.arm.setDefaultCommand(robot.armStop);
        // Driving
        driving = new SequentialCommandGroup();
        driving.addCommands(
                new TimerCommand(delay),
                robot.wristCenter,
                new MotorByEncoder(
                        robot.mwrist,
                        7250
                ),
                new MotorByEncoder(
                        robot.arm,
                        3650
                ),
                new DriveSeconds(
                        robot.drive,
                        515,
                        "up",
                        robot.imu,
                        false
                ),
                new RotateDrive(
                        robot.drive,
                        robot.imu,
                        0
                ),new TimerCommand(800),
                new DriveSeconds(
                        robot.drive,
                        350,
                        "right",
                        robot.imu,
                        false
                ),
                new RotateDrive(
                        robot.drive,
                        robot.imu,
                        0
                ),new TimerCommand(800),
                new DriveSeconds(
                        robot.drive,
                        160,
                        "up",
                        robot.imu,
                        false
                ),
                new RotateDrive(
                        robot.drive,
                        robot.imu,
                        0
                ),
                new TimerCommand(1500),
                new MotorByEncoder(
                        robot.mwrist,
                        6250
                ),
                new TimerCommand(delay),
                new MotorByEncoder(
                        robot.arm,
                        3600
                ),
                new TimerCommand(1500),
                new DriveSeconds(
                        robot.drive,
                        250,
                        "down",
                        robot.imu,
                        false
                ),
                new TimerCommand(1000),
                //Eject
                new CRServoBackwardDualTimed(

                        robot.intake,
                        robot.intake2,
                        1000
                ),
                new TimerCommand(500),
                new MotorByEncoder(
                        robot.arm,
                        200
                ),
                new DriveSeconds(
                        robot.drive,
                        100,
                        "down",
                        robot.imu,
                        false
                ),
                new SequentialCommandGroup(
                        new DriveSeconds(
                        robot.drive,
                        1900,
                        "left",
                        robot.imu,
                        false
                ),
                        new MotorByEncoder(
                                robot.arm,
                                2000
                        )
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
