package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class RobotSetup {
    public MotorSubsystem hang, arm;
    public MotorRaise hangRaise, armRaise;
    public MotorLower hangLower, armLower;
    public MotorStop hangStop, armStop;
    public CRServoSubsystem intake;
    public CRServoForward intakeForward;
    public CRServoBackward intakeBackward;
    public CRServoStop intakeStop;
    public ServoSubsystem launcher, cover;
    public LauncherCommand launcherRelease;
    public LEDSubsystem led;
    public LEDSetPattern c_alliance, c_chase;
    public DigitalLEDSubsystem d1;
    public DistanceSensorSubsystem dist1;



    public RobotSetup(
            String alliance,
            HardwareMap hmap
    ) {

        //LED
        led = new LEDSubsystem(hmap,"LIGHTS");
        c_alliance = new LEDSetPattern(led,getAlliance(alliance));
        c_chase = new LEDSetPattern(led, getChase(alliance));
        led.setDefaultCommand(c_alliance);


        //DigitalLED
        d1 = new DigitalLEDSubsystem(hmap, "GREEN", "RED");
        d1.setChannels(0);

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



        //Intake
        intake = new CRServoSubsystem(
                hmap,
                "GRIPPER",
                1.0,
                -1.0
        );
        intakeForward = new CRServoForward(intake);
        intakeBackward = new CRServoBackward(intake);
        intakeStop = new CRServoStop(intake);
        intake.setDefaultCommand(intakeStop);



        //Launcher
        launcher = new ServoSubsystem(hmap, "DRONE");
        cover = new ServoSubsystem(hmap, "COVER");
        launcherRelease = new LauncherCommand(
                launcher,
                cover,
                0.05,
                0.1,
                0.1,
                500
        );



        //DistanceSensor
        dist1 = new DistanceSensorSubsystem(hmap, "DISTANCE");


    }
    private RevBlinkinLedDriver.BlinkinPattern getAlliance(String alliance){
        if (alliance == "red") {
            return RevBlinkinLedDriver.BlinkinPattern.RED;
        } else if (alliance == "blue") {
            return RevBlinkinLedDriver.BlinkinPattern.BLUE;
        } else {
            return RevBlinkinLedDriver.BlinkinPattern.GRAY;
        }
    }
    private RevBlinkinLedDriver.BlinkinPattern getChase(String alliance){
        if (alliance == "red") {
            return RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
        } else if (alliance == "blue") {
            return RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
        } else {
            return RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY;
        }
    }
}
