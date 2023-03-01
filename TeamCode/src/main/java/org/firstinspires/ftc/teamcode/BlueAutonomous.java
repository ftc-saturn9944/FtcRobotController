package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueAutonomous extends CommandOpMode {

    static boolean FIELD_CENTRIC = true;

    private GamepadEx driverOp, toolOp;
    private GripperSubsystem gripper;
    private Button m_grabButton, m_releaseButton;
    private GripperGrab m_grabCommand;
    private GripperRelease m_releaseCommand;

    private MecanumSubsystem drive;
    private DefaultDrive m_driveCommand;

    private LiftSubsystem lift;
    private Button r_liftButton, l_liftButton;
    private Button field_toggleButton;
    private RaiseLift r_liftCommand;
    private LowerLift l_liftCommand;
    private StopLift s_liftCommand;

    private WristSubsystem wrist;
    private Button r_wristButton;
    private RotateWrist r_wristCommand;

    private CameraSubsystem camera;
    private ElapsedTime timer;

    private RevIMU imu;
    public void initialize() {

        imu = new RevIMU(hardwareMap);
        imu.init();
        SensorRevTOFDistance liftDistance = new SensorRevTOFDistance(hardwareMap, "LIFTDISTANCE");
        ServoEx gripServo = new SimpleServo(hardwareMap, "GRIPPER", 0, 90);
        ServoEx wristServo = new SimpleServo(hardwareMap, "WRIST", 0, 180);
        drive = new MecanumSubsystem(
                new MotorEx(hardwareMap, "RIGHTREAR", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "LEFTREAR", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "RIGHTFRONT", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "LEFTFRONT", Motor.GoBILDA.RPM_435),
                imu,
                false
        );
        gripper = new GripperSubsystem(gripServo);

        lift = new LiftSubsystem(hardwareMap, "LIFTMOTOR");

        wrist = new WristSubsystem(wristServo);

        telemetry.addData("GripperPosition", gripper::getPosition);
        //telemetry.addData("LiftSensor", lift::getDistance);
        telemetry.addData("LiftTargetName", lift::getTargetName);
        telemetry.addData("LiftTargetDist", lift::getTargetDist);
        telemetry.update();
        register(gripper, drive, lift, wrist);

        timer = new ElapsedTime();
        camera = new CameraSubsystem(hardwareMap, "Webcam 1");
        camera.initializeCamera();


        SequentialCommandGroup driving = new SequentialCommandGroup();
        ParallelRaceGroup cameraLookup = new ParallelRaceGroup();
        cameraLookup.addCommands(
                new TimerCommand(5000),
                new SignalDetection(camera)
        );
        driving.addRequirements(drive, gripper, wrist);
        driving.addCommands(
                cameraLookup,
                new GripperGrab(gripper),
                new DriveSeconds(drive,0, "stop", imu, false),
                new DriveSeconds(drive, 400, "right", imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new AutonomousForward(drive, camera, imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new AutonomousReverse(drive, camera, imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new DriveFromCamera(drive, camera, imu, false, "right"),
                new DriveSeconds(drive, 0, "stop", imu, false)
        );
        schedule(driving);
        //DriveTest driving = new DriveTest(drive, gripper, imu);
        //schedule(park);

    }

    @Override
    public void run() {
        telemetry.clearAll();
        telemetry.addData("ElapsedTime: ", timer.seconds());
        telemetry.addData("Target", camera::getColor);
        telemetry.update();
        super.run();
    }
}
