package org.firstinspires.ftc.teamcode;

import android.graphics.Camera;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueAutonomousTerminal2 extends CommandOpMode {

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
    private DriveSeconds s_driveCommand;

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

        lift = new LiftSubsystem(hardwareMap, "LIFTDISTANCE", "LIFTMOTOR");

        wrist = new WristSubsystem(wristServo);

        telemetry.addData("GripperPosition", gripper::getPosition);
        telemetry.addData("LiftSensor", lift::getDistance);
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
        driving.addRequirements(drive, gripper, wrist, lift);
        driving.addCommands(
                //cameraLookup,
                new GripperGrab(gripper),
                new LiftByEncoder(lift, 200),
                new DriveSeconds(drive,0, "stop", imu, false),
                new DriveSeconds(drive, 2500, "left", imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new DriveSeconds(drive, 1400, "up", imu, false, 0.3),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new LiftByEncoder(lift, 15000),
                new GripperRelease(gripper),
                new TimerCommand(500),
                //new StopLift(lift),
                new DriveSeconds(drive, 200, "down", imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new LiftByEncoder(lift, 100)
        );
        schedule(driving);

        s_liftCommand = new StopLift(lift);
        lift.setDefaultCommand(s_liftCommand);
        s_driveCommand = new DriveSeconds(drive, 0, "stop", imu, false);
        //drive.setDefaultCommand(s_driveCommand);


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
