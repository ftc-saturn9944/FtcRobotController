package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class SystemsTest extends CommandOpMode {

    private GamepadEx driverOp, toolOp;

    private Button leftFront, leftRear, rightFront, rightRear;
    private MotorEx lfront, lrear, rfront, rrear;
    public void initialize() {
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        lfront = new MotorEx(hardwareMap, "LEFTFRONT", Motor.GoBILDA.RPM_312);
        lrear = new MotorEx(hardwareMap, "LEFTREAR", Motor.GoBILDA.RPM_312);
        rfront = new MotorEx(hardwareMap, "RIGHTFRONT", Motor.GoBILDA.RPM_312);
        rrear = new MotorEx(hardwareMap, "RIGHTREAR", Motor.GoBILDA.RPM_312);

        leftFront = new GamepadButton(driverOp, GamepadKeys.Button.X);
        leftFront.whileHeld(new InstantCommand(() -> {
                    lfront.set(0.5);
                }));
        leftFront.whenReleased(new InstantCommand(() -> {
            lfront.stopMotor();
        }));
        leftRear = new GamepadButton(driverOp, GamepadKeys.Button.A);
        leftRear.whileHeld(new InstantCommand(() -> {
            lrear.set(0.5);
        }));
        leftRear.whenReleased(new InstantCommand(() -> {
            lrear.stopMotor();
        }));
        rightFront = (new GamepadButton(driverOp, GamepadKeys.Button.Y))
                .whileHeld(new InstantCommand(() -> {
                    rfront.set(0.5);
                }))
                .whenReleased(new InstantCommand(() -> {
                    rfront.stopMotor();
                }));
        rightRear = (new GamepadButton(driverOp, GamepadKeys.Button.B))
                .whileHeld(new InstantCommand(() -> {
                    rrear.set(0.5);
                }))
                .whenReleased(new InstantCommand(() -> {
                    rrear.stopMotor();
                }));

    }
}
