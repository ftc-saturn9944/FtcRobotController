package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Target Detection", group="Testing")
public class DetectionTesting extends CommandOpMode {

    // Driving
    private RevIMU imu;
    static Boolean FIELD_CENTRIC = true;
    private MecanumSubsystem drive;
    private DefaultDrive m_driveCommand;
    private DriveSeconds m_driveSecondsCommand;

    // Targeting
    private DistanceSubsystem distance;
    private FindTarget target3, target2;

    // Lift
    private LiftSubsystem lift;
    private RaiseLift m_raiseLiftCommand;
    private LowerLift m_lowerLiftCommand;
    private StopLift m_stopLiftCommand;

    // Automation
    private SequentialCommandGroup driving;

    public void initialize() {
        telemetry.setAutoClear(false);
        telemetry.addLine("-- Target Detection --");
        telemetry.addLine("Starting Initialization");
        telemetry.update();

        // IMU
        telemetry.addLine(" .. IMU");
        telemetry.update();
        imu = new RevIMU(hardwareMap);
        imu.init();

        // Drive System
        telemetry.addLine(" .. Drive");
        telemetry.update();
        drive = new MecanumSubsystem(
                new MotorEx(hardwareMap, "RIGHTREAR", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "LEFTREAR", Motor.GoBILDA.RPM_435),

                new MotorEx(hardwareMap, "RIGHTFRONT", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "LEFTFRONT", Motor.GoBILDA.RPM_435),
                imu,
                false
        );

        // Target System
        telemetry.addLine(" .. Target");
        telemetry.update();
        distance = new DistanceSubsystem(hardwareMap, "DISTANCE");


        // Lift System
        telemetry.addLine(" .. Lift");
        telemetry.update();
        lift = new LiftSubsystem(hardwareMap, "LIFTMOTOR");
        m_lowerLiftCommand = new LowerLift(lift);
        m_raiseLiftCommand = new RaiseLift(lift);
        m_stopLiftCommand = new StopLift(lift);
        lift.setDefaultCommand(m_stopLiftCommand);

        driving = new SequentialCommandGroup();
        driving.addCommands(
                new StopLiftAutonomous(lift),
                new ParallelDeadlineGroup(
                        new TargetDetected(distance),
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        new TimerCommand(250),
                                        new FindTarget(distance, DistanceSubsystem.Targets.Right, 90)
                                ),
                                new DriveSeconds(drive, 500, "left", imu, false),
                                new DriveSeconds(drive, 0, "stop", imu, false),
                                new ParallelRaceGroup(
                                        new TimerCommand(250),
                                        new FindTarget(distance, DistanceSubsystem.Targets.Center, 150)
                                ),
                                new InstantCommand(() -> {distance.setTarget(DistanceSubsystem.Targets.Left);})
                        )
                ),
                new TimerCommand(250),
                new LiftByEncoder(lift, -500),
                new StopLiftAutonomous(lift)
        );
        schedule(driving);
    }

    @Override
    public void run() {
        telemetry.clearAll();
        telemetry.addData("Target",distance::getTarget);
        telemetry.addData("Distance", distance::getDistance);
        telemetry.update();
        super.run();
    }
}
