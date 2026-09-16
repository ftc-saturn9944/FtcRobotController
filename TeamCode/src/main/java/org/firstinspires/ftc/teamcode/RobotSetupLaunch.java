package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotSetupLaunch {
    public RevIMU imu;
    public static boolean FIELD_CENTRIC = true;
    public MecanumSubsystem drive;
    public DefaultDrive driveCommand;
    public DualMotorSubsystem launch;
    public DualMotorRaise launchRaise;
    public DualMotorLower launchLower;
    public DualMotorStop launchStop;
    public ServoSubsystem trigger;
    public ServoSetPosition triggerRelease, triggerHold;
    public MotorSubsystem ramp;
    public MotorRaise rampRaise;
    public MotorLower rampLower;
    public MotorStop rampStop;
    public CRServoSubsystem crServo1, crServo2;
    public CRServoForwardDual intakeForward ;
    public CRServoBackwardDual intakeBackward;
    public CRServoStopDual intakeStop;

    public MotorEx frontLeft, frontRight, backLeft, backRight;


    public RobotSetupLaunch(HardwareMap hmap
    ) {
    //Drive
    imu = new RevIMU(hmap);
    imu.init();

    frontLeft = new MotorEx(hmap, "LEFTFRONT", Motor.GoBILDA.RPM_435);
    frontRight = new MotorEx(hmap, "RIGHTFRONT", Motor.GoBILDA.RPM_435);
    backRight = new MotorEx(hmap, "RIGHTREAR", Motor.GoBILDA.RPM_435);
    backLeft = new MotorEx(hmap, "LEFTREAR", Motor.GoBILDA.RPM_435);
    drive = new MecanumSubsystem(
            frontLeft,
            frontRight,
            backLeft,
            backRight,
            imu,
            false
    );

    //Launcher
    launch = new DualMotorSubsystem(
            hmap,
            "LAUNCHMOTOR1","LAUNCHMOTOR2"
            ,-0.70, 0.70, 0,
            //long dist: .7
            // short dist: .
            Motor.GoBILDA.BARE,
            Motor.ZeroPowerBehavior.BRAKE
    );
    launchRaise = new DualMotorRaise(launch);
    launchLower = new DualMotorLower(launch);
    launchStop = new DualMotorStop(launch);
    //launch.setDefaultCommand(launchStop);

    //Trigger
    trigger = new ServoSubsystem(
            hmap, "TRIGGER"
    );
    triggerRelease = new ServoSetPosition(trigger, .65);
    triggerHold = new ServoSetPosition(trigger, 0.1);

    //Ramp
    ramp = new MotorSubsystem(
            hmap,
            "RAMPMOTOR",
            1, -1, 0,
            Motor.GoBILDA.RPM_1620,
            Motor.ZeroPowerBehavior.BRAKE
    );
    rampRaise = new MotorRaise(ramp);
    rampLower = new MotorLower(ramp);
    rampStop = new  MotorStop(ramp);

    crServo1 = new CRServoSubsystem(
            hmap,
            "CRSERVO1",
            1.0,-1.0
    );
    crServo2 = new CRServoSubsystem(
            hmap,
            "CRSERVO2",
            1.0,
            -1.0
    );
    intakeForward = new CRServoForwardDual(crServo1, crServo2);
    intakeBackward = new CRServoBackwardDual(crServo1, crServo2);
    intakeStop = new CRServoStopDual(crServo1, crServo2);
    }
}
