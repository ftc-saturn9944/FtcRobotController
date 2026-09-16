package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Field Centric")
public class RobotTeleopFieldCentric extends CommandOpMode {
    private Button imuReset;
    private DefaultDrive driveCommand;
    private GamepadEx driverOp;
    private Button launcherRaise,launcherStop;
    private Button triggerHold,triggerRelease;
    private Button rampRaise,rampStop,rampLower;
    private Button intakeForward, intakeBackward, intakeStop;
    private RobotSetupLaunch robot;

    public void initialize() {
        robot = new RobotSetupLaunch(
                hardwareMap
        );
        telemetry.setAutoClear(false);
        telemetry.update();
        telemetry.addLine("startInitialization");
        telemetry.update();
        driverOp = new GamepadEx(gamepad1);

        //Driving
        imuReset = (new GamepadButton(driverOp, GamepadKeys.Button.Y))
                .whenPressed(
                        new InstantCommand(() -> robot.imu.reset())
                );
        driveCommand = new DefaultDrive(
                robot.drive,
                () -> driverOp.getLeftX(),
                () -> driverOp.getLeftY(),
                () -> driverOp.getRightX(),
                robot.imu,
                true
        );
        register(robot.drive);
        robot.drive.setDefaultCommand(driveCommand);

        //Launcher
        launcherRaise = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER);
        launcherStop = new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER);

        launcherRaise.whenHeld(robot.launchRaise);
        launcherStop.whenPressed(robot.launchStop);

        //Trigger
        triggerRelease = new GamepadButton(driverOp, GamepadKeys.Button.B);
        triggerHold = new GamepadButton(driverOp, GamepadKeys.Button.B);
        triggerRelease.whenPressed(robot.triggerRelease);
        triggerHold.whenReleased(robot.triggerHold);


        //Ramp
     rampRaise = new GamepadButton(driverOp, GamepadKeys.Button.X);
        rampStop = new GamepadButton(driverOp, GamepadKeys.Button.Y);
        rampLower = new GamepadButton(driverOp, GamepadKeys.Button.A);

        //rampRaise.whenPressed(//        robot.rampRaise
       // );
        rampLower.whenPressed(
                robot.rampLower
        );
        rampStop.whenPressed(
                new ParallelCommandGroup(
                        robot.rampStop, robot.intakeStop
                )
        );
        rampRaise.whenPressed(
                new ParallelCommandGroup(
                        robot.intakeForward, robot.rampRaise
                )
        );

        //new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN)
        //        .whenPressed(robot.intakeBackward);



    }
    @Override
    public void run(){
        telemetry.clearAll();
        telemetry.update();
        super.run();
    }
}
