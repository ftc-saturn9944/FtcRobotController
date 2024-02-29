package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class SystemsTest extends CommandOpMode {
    private GamepadEx driverOp,toolOp;
    private MotorSubsystem hang,arm;
    private Button hangRaise,hangLower;
    private Button armRaise,armLower;
    private MotorRaise c_hangRaise,c_armRaise;
    private MotorLower c_hangLower,c_armLower;
    private MotorStop c_hangStop,c_armStop;
    private CRServoSubsystem intake;
    private Button intakeForward,intakeBackward;
    private CRServoForward c_intakeForward;
    private CRServoBackward c_intakeBackward;
    private CRServoStop c_intakeStop;

    public void initialize() {
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);
        //Hanging
        hang = new MotorSubsystem(
                hardwareMap,
                "HANGMOTOR",
                0.8,-0.5,0,
                Motor.GoBILDA.RPM_117,
                Motor.ZeroPowerBehavior.BRAKE
        );

        // Explicit variables to implement commands tied to buttons
        hangRaise = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP);
        hangLower = new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN);
        c_hangRaise = new MotorRaise(hang);
        c_hangLower = new MotorLower(hang);
        hangRaise.whenHeld(c_hangRaise);
        hangLower.whenHeld(c_hangLower);

        /*
        // Equivalent to the lines above without extra variable definitions
        (new GamepadButton(driverOp, GamepadKeys.Button.DPAD_UP)).whenHeld(new MotorRaise(hang));
        (new GamepadButton(driverOp, GamepadKeys.Button.DPAD_DOWN)).whenHeld(new MotorLower(hang));
        */

        c_hangStop = new MotorStop(hang);
        hang.setDefaultCommand(c_hangStop);

        //Arm
        arm = new MotorSubsystem(
                hardwareMap,
                "LIFTMOTOR",
                1.0, -1.0,.05,
                Motor.GoBILDA.RPM_117,
                Motor.ZeroPowerBehavior.BRAKE
        );
        armRaise = new GamepadButton(driverOp, GamepadKeys.Button.RIGHT_BUMPER);
        armLower = new GamepadButton(driverOp, GamepadKeys.Button.LEFT_BUMPER);
        c_armRaise = new MotorRaise(arm);
        c_armLower = new MotorLower(arm);
        c_armStop = new MotorStop(arm);
        armRaise.whenHeld(c_armRaise);
        armLower.whenHeld(c_armLower);
        arm.setDefaultCommand(c_armStop);

        //Intake
        intake = new CRServoSubsystem(hardwareMap,"GRIPPER",1.0,-1.0);
        intakeForward = new GamepadButton(driverOp, GamepadKeys.Button.Y);
        intakeBackward = new GamepadButton(driverOp, GamepadKeys.Button.B);
        c_intakeForward = new CRServoForward(intake);
        c_intakeBackward = new CRServoBackward(intake);
        c_intakeStop = new CRServoStop(intake);
        intakeForward.whenHeld(c_intakeForward);
        intakeBackward.whenHeld(c_intakeBackward);
        intake.setDefaultCommand(c_intakeStop);
    }
}
