package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RobotTestTeleop extends CommandOpMode {
    private Button imuReset;
    private DefaultDrive driveCommand;
    private GamepadEx driverOp;
    private RobotSetupTesting robot;

    public void initialize() {
        robot = new RobotSetupTesting(
                hardwareMap
        );
        telemetry.setAutoClear(false);
        telemetry.update();
        telemetry.addLine("startInitialization");
        telemetry.update();
        driverOp = new GamepadEx(gamepad1);

        driveCommand = new DefaultDrive(
                robot.drive,
                () -> -driverOp.getLeftX(),
                () -> driverOp.getLeftY(),
                () -> driverOp.getRightX(),
                robot.imu,
                false
        );
        register(robot.drive);
        robot.drive.setDefaultCommand(driveCommand);

        (new GamepadButton(driverOp, GamepadKeys.Button.X))
                .whileHeld(new InstantCommand(() -> robot.frontLeft.set(1.0)))
                .whenReleased(new InstantCommand(() -> robot.frontLeft.set(0.0)));
        (new GamepadButton(driverOp, GamepadKeys.Button.Y))
                .whileHeld(new InstantCommand(() -> robot.frontRight.set(1.0)))
                .whenReleased(new InstantCommand(() -> robot.frontRight.set(0.0)));
        (new GamepadButton(driverOp, GamepadKeys.Button.A))
                .whileHeld(new InstantCommand(() -> robot.backLeft.set(1.0)))
                .whenReleased(new InstantCommand(() -> robot.backLeft.set(0.0)));
        (new GamepadButton(driverOp, GamepadKeys.Button.B))
                .whileHeld(new InstantCommand(() -> robot.backRight.set(1.0)))
                .whenReleased(new InstantCommand(() -> robot.backRight.set(0.0)));
    }
    @Override
    public void run(){
        telemetry.clearAll();
        telemetry.update();
        super.run();
    }
}
