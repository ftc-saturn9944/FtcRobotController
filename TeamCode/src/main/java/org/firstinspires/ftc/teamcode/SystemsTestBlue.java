package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SystemsTestBlue extends CommandOpMode {
    private GamepadEx driverOp,toolOp;
    private Button hangRaise,hangLower;
    private Button armRaise,armLower;
    private Button intakeForward,intakeBackward;
    private Button launcherRelease;
    private RobotSetup robot;

    public void initialize() {
        robot = new RobotSetup(
                "blue",
                hardwareMap
        );
        telemetry.setAutoClear(false);
        telemetry.update();
        telemetry.addLine("startInitialization");
        telemetry.update();
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);


        //Hanging
        // Explicit variables to implement commands tied to buttons
        hangRaise = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP);
        hangLower = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN);
        hangRaise.whenHeld(robot.hangRaise);
        hangLower.whenHeld(robot.hangLower);
        /*
        // Equivalent to the lines above without extra variable definitions
        (new GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP)).whenHeld(new MotorRaise(hang));
        (new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN)).whenHeld(new MotorLower(hang));
        */


        //Arm

        armRaise = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER);
        armLower = new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER);
        armRaise.whenHeld(robot.armRaise);
        armLower.whenHeld(robot.armLower);


        //Intake
        intakeForward = new GamepadButton(driverOp, GamepadKeys.Button.Y);
        intakeBackward = new GamepadButton(driverOp, GamepadKeys.Button.B);
        intakeForward.whenHeld(robot.intakeForward);
        intakeBackward.whenHeld(robot.intakeBackward);


        //Launcher
        robot.cover.setPosition(0.05); //set to closed by default at initialization
        launcherRelease = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        launcherRelease.whenPressed(robot.launcherRelease);
    }
    @Override
    public void run(){
        telemetry.clearAll();
        telemetry.addData("cover",robot.cover::getPosition);
        telemetry.addData("coverTarget",robot.launcherRelease::coverTarget);

        telemetry.update();
        super.run();

    }
}
