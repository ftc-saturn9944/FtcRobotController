package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class RunTeleop extends CommandOpMode {
    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static boolean FIELD_CENTRIC = false;

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

    private RevIMU imu;
    public void initialize() {
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        imu = new RevIMU(hardwareMap);
        imu.init();
        SensorRevTOFDistance liftDistance = new SensorRevTOFDistance(hardwareMap, "LIFTDISTANCE");
        ServoEx gripServo = new SimpleServo(hardwareMap, "GRIPPER", 0, 90);
        ServoEx wristServo = new SimpleServo(hardwareMap, "WRIST", 0, 180);
        drive = new MecanumSubsystem(
                new MotorEx(hardwareMap, "LEFTREAR", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "RIGHTREAR", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "LEFTFRONT", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "RIGHTFRONT", Motor.GoBILDA.RPM_435),
                imu,
                false
        );
        gripper = new GripperSubsystem(gripServo);
        m_grabCommand = new GripperGrab(gripper);
        m_releaseCommand = new GripperRelease(gripper);
        m_grabButton = (new GamepadButton(toolOp, GamepadKeys.Button.B))
                .whenPressed(m_grabCommand);
        m_releaseButton = (new GamepadButton(toolOp, GamepadKeys.Button.Y))
                .whenPressed(m_releaseCommand);

        lift = new LiftSubsystem(hardwareMap, "LIFTDISTANCE", "LIFTMOTOR");
        l_liftCommand = new LowerLift(lift);
        r_liftCommand = new RaiseLift(lift);
        s_liftCommand = new StopLift(lift);
        r_liftButton = (new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER))
                .whileHeld(r_liftCommand)
                .whenReleased(s_liftCommand);
        l_liftButton = (new GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER))
                .whileHeld(l_liftCommand)
                .whenReleased(s_liftCommand);

        new GamepadButton(driverOp, GamepadKeys.Button.Y).whenPressed(
                drive::swapDriveMethod
        );

        lift.setDefaultCommand(s_liftCommand);

        wrist = new WristSubsystem(wristServo);
        r_wristCommand = new RotateWrist(wrist);
        r_wristButton = (new GamepadButton(toolOp, GamepadKeys.Button.X))
                .whenPressed(r_wristCommand);

        m_driveCommand = new DefaultDrive(
                drive,
                () -> driverOp.getLeftX(),
                () -> driverOp.getLeftY(),
                () -> -driverOp.getRightX(),
                imu,
                FIELD_CENTRIC
        );

        telemetry.addData("GripperPosition", gripper::getPosition);
        telemetry.addData("LiftSensor", lift::getDistance);
        telemetry.addData("LiftTargetName", lift::getTargetName);
        telemetry.addData("LiftTargetDist", lift::getTargetDist);
        telemetry.update();
        register(gripper, drive, lift, wrist);
        drive.setDefaultCommand(m_driveCommand);
    }

    @Override
    public void run() {
        telemetry.clearAll();
        telemetry.addData("GripperPosition", gripper::getPosition);
        telemetry.addData("LiftSensor", lift::getDistance);
        telemetry.addData("LiftTargetName", lift::getTargetName);
        telemetry.addData("LiftTargetDist", lift::getTargetDist);
        telemetry.addData("Driver Field Centric?", drive::getDriveMethod);
        telemetry.update();
        super.run();
    }
}
