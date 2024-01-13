package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red - Left - Park", group="Red", preselectTeleOp = "RobotTeleOpField")
public class RedLeftPark extends CommandOpMode {

    // Customize for TeamMate auto
    private int delayMilli = 7500;

    // Driving
    private RevIMU imu;
    static Boolean FIELD_CENTRIC = true;
    private MecanumSubsystem drive;
    private DefaultDrive m_driveCommand;
    private DriveSeconds m_driveSecondsCommand;

    // Intake
    private GripperSubsystem gripper;
    private GripperGrab m_grabCommand;
    private GripperRelease m_releaseCommand;
    private GripperStop m_gripperStop;

    // Targeting
    private DistanceSubsystem distance;
    private FindTarget target3, target2;

    // Lift
    private LiftSubsystem lift;
    private RaiseLift m_raiseLiftCommand;
    private LowerLift m_lowerLiftCommand;
    private StopLift m_stopLiftCommand;

    // Timer
    private ElapsedTime timer;

    // Automation
    private SequentialCommandGroup driving;
    public void initialize() {
        telemetry.setAutoClear(false);
        telemetry.addLine("-- Red Left Autonomous --");
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
        // Intake System
        telemetry.addLine(" .. Intake");
        telemetry.update();
        CRServo gripServo = new CRServo(hardwareMap, "GRIPPER");
        gripper = new GripperSubsystem(gripServo);
        m_grabCommand = new GripperGrab(gripper);
        m_releaseCommand = new GripperRelease(gripper);
        m_gripperStop = new GripperStop(gripper);
        //gripper.setDefaultCommand(m_gripperStop);

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

        timer = new ElapsedTime();

        // Done Init, now setup automation
        telemetry.addLine("Done");
        telemetry.addLine("Setup Automation");
        telemetry.update();

        driving = new SequentialCommandGroup();
        driving.addRequirements(gripper, lift, drive);
        driving.addCommands(
                new TimerCommand(delayMilli),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new LiftByEncoder(lift, -10),
                new StopLiftAutonomous(lift),
                new DriveSeconds(drive, 170, "down", imu, false),
                new RotateDrive(drive, -88.0),
                new DriveSeconds(drive, 3000, "down", imu, false),
                new RotateDrive(drive, 0.0),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new ParallelRaceGroup(
                        new TimerCommand(3000),
                        new GripperGrab(gripper)
                ),
                new GripperStop(gripper),
                new DriveSeconds(drive, 0, "stop", imu, false)
        );

        schedule(driving);
    }

    @Override
    public void run() {
        telemetry.clearAll();
        telemetry.addData("IMU Heading", imu.getHeading());
        telemetry.update();
        super.run();
    }
}
