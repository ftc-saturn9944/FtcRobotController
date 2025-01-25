package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Timer;

@Autonomous(name="Auto Right Dual", preselectTeleOp = "SystemsTestDualGripper")
public class AutoRightDual extends CommandOpMode {

    private RobotSetupDual robot;
    private long delay = 100;
    private SequentialCommandGroup driving, test;

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
        //Driving
//        driving = new SequentialCommandGroup();
//        driving.addCommands(
//                robot.wristScore,
//                new MotorByEncoder(
//                        robot.mwrist,
//                        2000
//                ),
//                new DriveSeconds(
//                        robot.drive,
//                        100,
//                        "up",
//                        robot.imu,
//                        false
//                ),
//                new RotateDrive(
//                        robot.drive,
//                        robot.imu,
//                        90
//                ),
//                new TimerCommand(500),
//                new DriveSeconds(
//                        robot.drive,
//                        900,
//                        "down",
//                        robot.imu,
//                        false
//                ),
//                new RotateDrive(
//                        robot.drive,
//                        robot.imu,
//                        0
//                )
//        );
        robot.mwrist.setDefaultCommand(robot.mwristStop);
        robot.arm.setDefaultCommand(robot.armStop);

        test = new SequentialCommandGroup();
        test.addCommands(
                new TimerCommand(delay),
                // Ensure servo is locked
                robot.wristCenter,
                // Lift arm to correct location
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
                ),
                new TimerCommand(800),
                new DriveSeconds(
                        robot.drive,
                        675,
                        "left",
                        robot.imu,
                        false
                ),
                new RotateDrive(
                        robot.drive,
                        robot.imu,
                        0
                ),
                new TimerCommand(800),
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
                        6300
                ),
                new TimerCommand(delay),
                new MotorByEncoder(
                        robot.arm,
                        3600
                ),
                new TimerCommand(1500),
                new DriveSeconds(
                        robot.drive,
                        300,
                        "down",
                        robot.imu,
                        false
                ),
                new TimerCommand(1000),
                // Eject, Eject, Eject!!!
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
                new RotateDrive(
                        robot.drive,
                        robot.imu,
                        90
                ),
                new TimerCommand(500),
                new DriveSeconds(
                        robot.drive,
                        1200,
                        "down",
                        robot.imu,
                        false
                )


        );
        schedule(test);
        /*
         forward,left,
         High Bracket,
         move pieces from right side to obs. zone
          park
         */
    }

    @Override
    public void run(){
        telemetry.clearAll();
        telemetry.addData("Arm Rotation Enc", robot.mwrist::getEncoder);
        telemetry.addData("Arm Extension Enc", robot.arm::getEncoder);
        //telemetry.addData("Motors", robot.drive::getEncoders);
        telemetry.update();
        super.run();

    }
}
