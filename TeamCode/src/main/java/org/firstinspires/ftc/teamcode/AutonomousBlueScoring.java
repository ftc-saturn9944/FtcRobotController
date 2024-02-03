package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.HashMap;

@Autonomous(name="Blue Autonomous - Detection", group="A-Full Score")
public class AutonomousBlueScoring extends CommandOpMode {

    // Tuning Variables
    // "up" or "down".  Backwards so "up" means towards the wall/human player
    private String parkDir = "up";
    // How many milliseconds to move for park purposes.
    // After tuning comment/uncomment the 0 and actual value to switch from staying put to parking
    private long parkDuration = 900;
    // private long parkDuration = 0;
    // Delay before starting to account for alliance partner
    private long delay = 0;

    // Driving
    private RevIMU imu;
    static Boolean FIELD_CENTRIC = true;
    private MecanumSubsystem drive;
    private DefaultDrive m_driveCommand;
    private DriveSeconds m_driveSecondsCommand;

    // Targeting
    private DistanceSubsystem distance;
    private FindTarget target3, target2;

    private GripperSubsystem gripper;

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

        // Intake
        telemetry.addLine(" .. Intake");
        telemetry.update();
        CRServo gripServo = new CRServo(hardwareMap, "GRIPPER");
        gripper = new GripperSubsystem(gripServo);

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
                new TimerCommand(delay),
                new DriveSeconds(drive, 100, "down", imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false),
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
                // TODO: Post Detection adjustments
                // Center in starting tile.  Right has to move over, left and center moved too far
                new ConditionalCommand(
                        // When detecting right adjust this
                        new DriveSeconds(drive, 250, "left", imu, false), // if true
                        // When detected left/center adjust this
                        new DriveSeconds(drive, 350, "right", imu, false), // if false
                        () -> distance.getTarget() == DistanceSubsystem.Targets.Right
                ),
                new DriveSeconds(drive, 0, "stop", imu, false),
                // How far forward to center in spike marks
                new DriveSeconds(drive, 1000, "down", imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false),
                // TODO: We should now be centered between the spike marks
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                                put(DistanceSubsystem.Targets.Left, new SequentialCommandGroup(
                                        new RotateDrive(drive, -90.0),
                                        new DriveSeconds(drive, 0, "stop", imu, false),
                                        // Back up to put intake at left spike mark
                                        new DriveSeconds(drive, 450, "up", imu, false),
                                        new DriveSeconds(drive, 0, "stop", imu, false),
                                        new RotateDrive(drive, -150),
                                        new DriveSeconds(drive, 0, "stop", imu, false),
                                        new RotateDrive(drive, -90),
                                        new DriveSeconds(drive, 0, "stop", imu, false)
                                ));
                                put(DistanceSubsystem.Targets.Center, new SequentialCommandGroup(
                                        // Bump the prop out of the way and return
                                        // Move forward to push the prop
                                        new DriveSeconds(drive, 200, "down", imu, false),
                                        new DriveSeconds(drive, 0, "stop", imu, false),
                                        // Back up to align intake
                                        new DriveSeconds(drive, 200, "up", imu, false),
                                        new DriveSeconds(drive, 0, "stop", imu, false)
                                ));
                                put(DistanceSubsystem.Targets.Right, new SequentialCommandGroup(
                                        // Face right spike, bump prop and return
                                        new RotateDrive(drive, -90.0),
                                        new DriveSeconds(drive, 0, "stop", imu, false),
                                        new DriveSeconds(drive, 400, "down", imu, false),
                                        new DriveSeconds(drive, 0, "stop", imu, false),
                                        new DriveSeconds(drive, 100, "up", imu, false),
                                        new DriveSeconds(drive, 0, "stop", imu, false)
                                ));
                            }},
                        distance::getTarget
                ),
                // Deposit 1 pixel
                new ParallelRaceGroup(
                        new TimerCommand(1500), // Adjust to only be 1 pixel
                        new GripperGrab(gripper)
                ),
                new GripperStop(gripper),
                // Center on square 1 closer to backdrop than spike marks
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(DistanceSubsystem.Targets.Left, new SequentialCommandGroup(
                                    // Back up slightly to center
                                    new DriveSeconds(drive, 150, "up", imu, false),
                                    new DriveSeconds(drive, 0, "stop", imu, false)
                            ));
                            put(DistanceSubsystem.Targets.Center, new SequentialCommandGroup(
                                    new DriveSeconds(drive, 100, "up", imu, false),
                                    new DriveSeconds(drive, 0, "stop", imu, false),
                                    // Turn to match all other alignments and then back up to center
                                    new RotateDrive(drive, -90.0),
                                    new DriveSeconds(drive, 0, "stop", imu, false),
                                    new DriveSeconds(drive, 300, "up", imu, false),
                                    new DriveSeconds(drive, 0, "stop", imu, false)
                            ));
                            put(DistanceSubsystem.Targets.Right, new SequentialCommandGroup(
                                    // Back up to center
                                    new DriveSeconds(drive, 320, "up", imu, false),
                                    new DriveSeconds(drive, 0, "stop", imu, false)
                            ));
                        }},
                        distance::getTarget
                ),
                // We are now centered 1 square closer towards the backdrop from the spike marks facing
                // away from the backdrop
                // At a *minimum* be aligned facing away from the backdrop but in line with the center
                // scoring mark
                new RotateDrive(drive, 87), // TODO: CHECK HEADING
                new DriveSeconds(drive, 0, "stop", imu, false),
                new ParallelRaceGroup(
                        new DriveSeconds(drive, 15000, "down", imu, false, 0.3),
                        new TargetProximity(distance, 30)
                ),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new LiftByEncoder(lift, -2500),
                new StopLiftAutonomous(lift),
                new TimerCommand(2000),
                new ParallelRaceGroup(
                        new DriveSeconds(drive, 15000, "down", imu, false, 0.15),
                        new TargetProximity(distance, 3.5)
                ),
                new DriveSeconds(drive, 0, "stop", imu, false),
                // Add strafing logic ...
                new SelectCommand(
                        new HashMap<Object, Command>() {{
                            put(DistanceSubsystem.Targets.Left, new SequentialCommandGroup(
                                    new DriveSeconds(drive, 200, "left", imu, false),
                                    new DriveSeconds(drive, 0, "stop", imu, false)
                            ));
                            // No Center since we are already in place
                            put(DistanceSubsystem.Targets.Right, new SequentialCommandGroup(
                                    new DriveSeconds(drive, 200, "right", imu, false),
                                    new DriveSeconds(drive, 0, "stop", imu, false)
                            ));
                        }},
                        distance::getTarget
                ),
                // End strafing logic ...
                new TimerCommand(2000),
                new ParallelRaceGroup(
                        new GripperGrab(gripper),
                        new TimerCommand(1500)
                ),
                new GripperStop(gripper),
                // Pull away from backdrop to not crash into it
                new DriveSeconds(drive, 200, "up", imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false),
                // Align with original heading for Driver comfort
                new RotateDrive(drive, 0),
                new DriveSeconds(drive, 0, "stop", imu, false),
                // Use tuning variables to adjust how far to park (and which direction)
                new DriveSeconds(drive, parkDuration, parkDir, imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false)
        );
        schedule(driving);
    }

    @Override
    public void run() {
        telemetry.clearAll();
        telemetry.addData("Target",distance::getTarget);
        telemetry.addData("Distance", distance::getDistance);
        telemetry.addData("Heading", imu.getHeading());
        telemetry.update();
        super.run();
    }
}
