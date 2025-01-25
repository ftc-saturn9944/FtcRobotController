package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SystemsTestDualGripper extends CommandOpMode {
    private Button imuReset;
    private DefaultDrive driveCommand;
    private GamepadEx driverOp,toolOp;
    private Button hangRaise,hangLower;
    private Button armRaise,armLower, armUp, armDown;
    private Button intakeForward,intakeBackward;
    private Button wristScore, wristCenter;
    private Button launcherRelease;
    private RobotSetupDual robot;

    public void initialize() {
        robot = new RobotSetupDual(
                "blue",
                hardwareMap
        );
        telemetry.setAutoClear(false);
        telemetry.update();
        telemetry.addLine("startInitialization");
        telemetry.update();
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        // Driving
        imuReset = (new GamepadButton(driverOp, GamepadKeys.Button.Y))
                .whenPressed(
                        new InstantCommand(() -> robot.imu.reset())
                );
        driveCommand = new DefaultDrive(
                robot.drive,
                () -> driverOp.getLeftX(),
                () -> driverOp.getLeftY(),
                () -> -driverOp.getRightX(),
                robot.imu,
                robot.FIELD_CENTRIC
        );
        register(robot.drive);
        robot.drive.setDefaultCommand(driveCommand);

        //Hanging
        // Explicit variables to implement commands tied to buttons
        hangRaise = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER);
        hangLower = new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER);
        hangRaise.whenHeld(robot.hangRaise);
        hangLower.whenHeld(robot.hangLower);
        /*
        // Equivalent to the lines above without extra variable definitions
        (new GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP)).whenHeld(new MotorRaise(hang));
        (new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN)).whenHeld(new MotorLower(hang));
        */


        //Arm
        armRaise = new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER);
        armLower = new GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER);
        armRaise.whenHeld(robot.armRaise);
        armLower.whenHeld(robot.armLower);

        //Arm Rotation
        armUp = new GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN);
        armDown = new GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP);
        armUp.whenHeld(robot.mwristRaise);
        armDown.whenHeld(robot.mwristLower);

        //Intake
        intakeForward = new GamepadButton(toolOp, GamepadKeys.Button.Y);
        intakeBackward = new GamepadButton(toolOp, GamepadKeys.Button.B);
        intakeForward.whenHeld(robot.intakeForward);
        intakeBackward.whenHeld(robot.intakeBackward);

        //Wrist
        wristCenter = new GamepadButton(toolOp, GamepadKeys.Button.A);
        wristScore = new GamepadButton(toolOp, GamepadKeys.Button.X);
        wristCenter.whenPressed(robot.wristCenter);
        wristScore.whenPressed(robot.wristScore);

//        robot.wrist.setPosition(0.45);

    }
    @Override
    public void run(){
        telemetry.clearAll();
        telemetry.addData("Arm Rotation Enc", robot.mwrist::getEncoder);
        telemetry.addData("Arm Extension Enc", robot.arm::getEncoder);
        telemetry.addData("LFront Enc", robot.drive::getEncoderLFront);
        telemetry.addData("LRear Enc", robot.drive::getEncoderLRear); // Center turn
        telemetry.addData("RFront Enc", robot.drive::getEncoderRFront); // Right side
        telemetry.addData("RRear Enc", robot.drive::getEncoderRRear); // left side
        telemetry.update();
        super.run();

    }
}
