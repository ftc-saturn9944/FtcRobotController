package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RobotTeleOpField", group="CenterStage")
public class RobotTeleOpField extends CommandOpMode {
    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static boolean FIELD_CENTRIC = true;

    private DistanceSubsystem distance;
    private FindTarget target;
    private GamepadEx driverOp, toolOp;
    private GripperSubsystem gripper;
    private Button m_grabButton, m_releaseButton, m_resetIMUButton;

    private Button m_raiseHang, m_lowerHang;
    private HangingSubsystem hang;
    private RaiseHang rHangCommand;
    private LowerHang lHangCommand;
    private HangStop hangStop;
    private GripperGrab m_grabCommand;
    private GripperRelease m_releaseCommand;
    private GripperStop m_gripperStop;

    private MecanumSubsystem drive;
    private DefaultDrive m_driveCommand;
    // private Button m_resetIMUButton;

    private LiftSubsystem lift;
    private Button r_liftButton, l_liftButton;
    private Button field_toggleButton;
    private RaiseLift r_liftCommand;
    private LowerLift l_liftCommand;
    private StopLift s_liftCommand;

    private LauncherSubsystem launcher;
    private LaunchDrone launch_command;
    private Button l_button;

    private RevIMU imu;

    public void initialize() {
        telemetry.setAutoClear(false);
        telemetry.addLine("Starting Initialization");
        telemetry.update();

        telemetry.addLine(" ... Gamepads");
        telemetry.update();
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        telemetry.addLine(" ... IMU");
        telemetry.update();
        imu = new RevIMU(hardwareMap);
        imu.init();
        m_resetIMUButton = (new GamepadButton(driverOp, GamepadKeys.Button.Y))
                .whenPressed(
                        new InstantCommand(() -> imu.reset())
                );

        m_resetIMUButton = (new GamepadButton(driverOp, GamepadKeys.Button.Y))
                .whenPressed(
                        new InstantCommand(() -> imu.reset())
                );

        telemetry.addLine(" ... Mecanum Wheels");
        telemetry.update();
        drive = new MecanumSubsystem(
                new MotorEx(hardwareMap, "RIGHTREAR", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "LEFTREAR", Motor.GoBILDA.RPM_435),

                new MotorEx(hardwareMap, "RIGHTFRONT", Motor.GoBILDA.RPM_435),
                new MotorEx(hardwareMap, "LEFTFRONT", Motor.GoBILDA.RPM_435),
                imu,
                false
        );
        m_driveCommand = new DefaultDrive(
                drive,
                () -> driverOp.getLeftX(),
                () -> driverOp.getLeftY(),
                () -> -driverOp.getRightX(),
                imu,
                FIELD_CENTRIC
        );

        register(drive);
        drive.setDefaultCommand(m_driveCommand);


        telemetry.addLine(" ... Intake");
        telemetry.update();
        CRServo gripServo = new CRServo(hardwareMap, "GRIPPER");
        gripper = new GripperSubsystem(gripServo);
        m_grabCommand = new GripperGrab(gripper);
        m_releaseCommand = new GripperRelease(gripper);
        m_gripperStop = new GripperStop(gripper);
        m_grabButton = (new GamepadButton(toolOp, GamepadKeys.Button.B))
                .whenPressed(m_grabCommand)
                .whenReleased(m_gripperStop);
        m_releaseButton = (new GamepadButton(toolOp, GamepadKeys.Button.Y))
                .whenPressed(m_releaseCommand)
                .whenReleased(m_gripperStop);

        telemetry.addLine(" ... Hanging");
        telemetry.update();
        m_raiseHang = new GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP);
        m_lowerHang = new GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN);

        hang = new HangingSubsystem(hardwareMap, "HANGMOTOR");
        rHangCommand = new RaiseHang(hang);
        lHangCommand = new LowerHang(hang);
        hangStop = new HangStop(hang);
        m_raiseHang.whenPressed(rHangCommand)
                .whenReleased(hangStop);
        m_lowerHang.whenPressed(lHangCommand)
                .whenReleased(hangStop);

        telemetry.addLine(" ... Lift");
        telemetry.update();
        lift = new LiftSubsystem(hardwareMap, "LIFTMOTOR");
        l_liftCommand = new LowerLift(lift);
        r_liftCommand = new RaiseLift(lift);
        s_liftCommand = new StopLift(lift);
        r_liftButton = (new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER))
                .whileHeld(r_liftCommand)
                .whenReleased(s_liftCommand);
        l_liftButton = (new GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER))
                .whileHeld(l_liftCommand)
                .whenReleased(s_liftCommand);
        lift.setDefaultCommand(s_liftCommand);

        telemetry.addLine(" ... Launcher");
        telemetry.update();
        launcher = new LauncherSubsystem(hardwareMap, "DRONE");
        launch_command = new LaunchDrone(launcher);
        l_button = (new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_STICK_BUTTON))
                .whenPressed(launch_command);

        telemetry.addLine(" ... Distance Sensor");
        telemetry.update();
        distance = new DistanceSubsystem(hardwareMap, "DISTANCE");
        target = new FindTarget(distance, DistanceSubsystem.Targets.Left, 50.0);
        distance.setDefaultCommand(target);
        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void run() {
        telemetry.clearAll();
        telemetry.addData("DISTANCE", distance::getDistance);
        telemetry.addData("Target", distance::getTarget);
        telemetry.addData("Lift", lift::getEncoderValue);
        telemetry.update();
        super.run();
    }
}