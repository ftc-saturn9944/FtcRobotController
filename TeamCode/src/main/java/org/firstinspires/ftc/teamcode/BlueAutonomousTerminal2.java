package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

@Autonomous(name="Right Start - Score Center", group="Center High")
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

    private EncoderDriveSubsystem encoderDrive;
    public void initialize() {

        imu = new RevIMU(hardwareMap);
        imu.init();
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
        encoderDrive = new EncoderDriveSubsystem(
                hardwareMap.get(DcMotor.class,"LEFTFRONT"),
                hardwareMap.get(DcMotor.class, "LEFTREAR"),
                hardwareMap.get(DcMotor.class, "RIGHTFRONT"),
                hardwareMap.get(DcMotor.class, "RIGHTREAR" )
        );
        gripper = new GripperSubsystem(gripServo);

        lift = new LiftSubsystem(hardwareMap, "LIFTMOTOR");

        wrist = new WristSubsystem(wristServo);

        telemetry.addData("GripperPosition", gripper::getPosition);
        //telemetry.addData("LiftSensor", lift::getDistance);
        telemetry.addData("LiftTargetName", lift::getTargetName);
        telemetry.update();
        register(gripper, drive, lift, wrist);

        timer = new ElapsedTime();
        camera = new CameraSubsystem(hardwareMap, "Webcam 1");
        camera.initializeCamera();

        /* Parking Spots */
        int park1 = 625;
        int park2 = 625+1250;
        int park3 = 625+1250+1250;

        HashMap<Object, Command> parkDrive = new HashMap<Object, Command>() {{
            put("Yellow", new DriveByEncoder(encoderDrive, park1, "Right", 0.8));
            put("Purple", new DriveByEncoder(encoderDrive, park2, "Right", 0.8));
            put("Green", new DriveByEncoder(encoderDrive, park3, "Right", 0.8));
            put("Park", new SequentialCommandGroup(
                    new DriveByEncoder(encoderDrive, 1200, "Down", 0.8),
                    new DriveByEncoder(encoderDrive, park3, "Right", 0.8)
            ));
        }};
        SelectCommand parkLocation = new SelectCommand(
                parkDrive,
                // Selector
                camera::getColor
        );
        SequentialCommandGroup driving = new SequentialCommandGroup();
        ParallelRaceGroup cameraLookup = new ParallelRaceGroup();
        cameraLookup.addCommands(
                new TimerCommand(5000),
                new SignalDetection(camera)
        );
        driving.addRequirements(encoderDrive, gripper, wrist, lift);
        driving.addCommands(
                new GripperGrab(gripper),
                new RotateWrist(wrist),
                cameraLookup,
                new LiftByEncoder(lift, -200),
                new StopLiftAutonomous(lift),

                /* Drive from Left */
                new DriveByEncoder(encoderDrive, 1680, "Left", 0.8),
                /* Drive from Right */
                //new DriveByEncoder(encoderDrive, 2100, "Right", 0.8),
                /* Line up to the high Junction */

                new DriveByEncoder(encoderDrive, 10, "Down", 0.2),
                new DriveByEncoder(encoderDrive, 700, "Up", 0.3),
                new TimerCommand(300),
                new DriveByEncoder(encoderDrive, 500, "Up", 0.3),

                //new TimerCommand(3000),


                /* Scoring */
                new LiftByEncoder(lift, -6850),
                new StopLiftAutonomous(lift),
                new DriveByEncoder(encoderDrive, 200, "Up", 0.2),
                new TimerCommand(100),
                new LiftByEncoder(lift, -6150),
                new StopLiftAutonomous(lift),
                new TimerCommand(500),
                new GripperRelease(gripper),
                new TimerCommand(500),
                new LiftByEncoder(lift, -6850),
                new StopLiftAutonomous(lift),
                new DriveByEncoder(encoderDrive, 200, "Down", 0.2),
                new GripperGrab(gripper),
                new RotateWrist(wrist),
                new LiftByEncoder(lift, -1500),
                new StopLiftAutonomous(lift),
                parkLocation,
                new StopLiftAutonomous(lift)
                //new DriveByEncoder(encoderDrive, 625, "Left", 0.5),
                //new TimerCommand(3000),
                //new DriveByEncoder(encoderDrive, 1250, "Left", 0.5),
                //new TimerCommand(3000),
                //new DriveByEncoder(encoderDrive, 1250, "Left", 0.5),


        /*cameraLookup,
                new GripperGrab(gripper),
                new RotateWrist(wrist),
                new LiftByEncoder(lift, 200),
                new DriveSeconds(drive,0, "stop", imu, false),
                new DriveSeconds(drive, 2700, "left", imu, false),//2700 worked at ~12.95 volts, 29850 had best results
                new DriveSeconds(drive, 0, "stop", imu, false),
                new DriveSeconds(drive, 1400, "up", imu, false, 0.3),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new LiftByEncoder(lift, 15000),

                new TimerCommand(1000),
                new DriveSeconds(drive, 600, "up", imu, false, 0.3),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new TimerCommand(500),
                new LiftByEncoder(lift, 13000),
                new GripperRelease(gripper),
                new LiftByEncoder(lift, 15000),
                new RotateWrist(wrist),
                new GripperGrab(gripper),
                new TimerCommand(500),
                new DriveSeconds(drive, 150, "down", imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new LiftByEncoder(lift, 100),
                //new StopLift(lift),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new DriveFromCamera(drive, camera, imu, false, "right"),
                new DriveSeconds(drive, 0, "stop", imu, false)
                //new StopLift(lift)
                //parkParallel
                */

                );
        schedule(driving);

        s_liftCommand = new StopLift(lift);
        //lift.setDefaultCommand(s_liftCommand);
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
