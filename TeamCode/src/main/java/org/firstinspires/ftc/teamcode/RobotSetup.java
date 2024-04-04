package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotSetup {
    public MotorSubsystem hang, arm;
    public MotorRaise hangRaise, armRaise;
    public MotorLower hangLower, armLower;
    public MotorStop hangStop, armStop;
    private CRServoSubsystem intake;
    public RobotSetup(
            Telemetry telemetry,
            String alliance,
            HardwareMap hmap
    ) {
        telemetry.clearAll();
        telemetry.addLine("Setting Robot up");
        telemetry.update();

        // Hanging
        hang = new MotorSubsystem(
                hmap,
                "HANGMOTOR",
                0.8, -0.5, 0,
                Motor.GoBILDA.RPM_117,
                Motor.ZeroPowerBehavior.BRAKE
        );
        hangRaise = new MotorRaise(hang);
        hangLower = new MotorLower(hang);
        hangStop = new MotorStop(hang);
        hang.setDefaultCommand(hangStop);
        telemetry.addLine("Hanging");
        telemetry.update();

        // Arm
        arm = new MotorSubsystem(
                hmap,
                "LIFTMOTOR",
                1.0, -1.0, .05,
                Motor.GoBILDA.RPM_117,
                Motor.ZeroPowerBehavior.BRAKE
        );
        armRaise = new MotorRaise(arm);
        armLower = new MotorLower(arm);
        armStop = new MotorStop(arm);
        arm.setDefaultCommand(armStop);
        telemetry.addLine("Arm");
        telemetry.update();
    }
}
