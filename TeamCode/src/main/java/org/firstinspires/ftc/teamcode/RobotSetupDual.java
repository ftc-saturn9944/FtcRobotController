package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotSetupDual {
    public RevIMU imu;
    public static boolean FIELD_CENTRIC = true;
    public MecanumSubsystem drive;
    public DefaultDrive driveCommand;
    public MotorSubsystem hang, arm, mwrist;
    public MotorRaise hangRaise, armRaise, mwristRaise;
    public MotorLower hangLower, armLower,mwristLower;
    public MotorStop hangStop, armStop, mwristStop;
    public CRServoSubsystem intake, intake2;
    public CRServoForwardDual intakeForward;
    public CRServoBackwardDual intakeBackward;
    public CRServoStopDual intakeStop;
    public ServoSubsystem wrist;
    public LauncherCommand launcherRelease;
    public LEDSubsystem led;
    public LEDSetPattern c_alliance, c_chase;
    public DigitalLEDSubsystem d1;
    public DistanceSensorSubsystem dist1;
    public ServoSetPosition wristCenter, wristScore;

    public RobotSetupDual(
            String alliance,
            HardwareMap hmap
    ) {

        //LED
        //led = new LEDSubsystem(hmap,"LIGHTS");
        //c_alliance = new LEDSetPattern(led,getAlliance(alliance));
        //led.setDefaultCommand(c_alliance);


        //DigitalLED
        //d1 = new DigitalLEDSubsystem(hmap, "GREEN", "RED");
        //d1.setChannels(0);

        // Drive
        imu = new RevIMU(hmap);
        imu.init();

        drive = new MecanumSubsystem(
                // left front
                new MotorEx(hmap, "RIGHTREAR", Motor.GoBILDA.RPM_435),
                // right front
                new MotorEx(hmap, "LEFTREAR", Motor.GoBILDA.RPM_435),
                // left rear
                new MotorEx(hmap, "RIGHTFRONT", Motor.GoBILDA.RPM_435),
                // right rear
                new MotorEx(hmap, "LEFTFRONT", Motor.GoBILDA.RPM_435),
                imu,
                false
        );

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
                1.0, -1.0, 0,
                Motor.GoBILDA.RPM_117,
                Motor.ZeroPowerBehavior.BRAKE
        );
        armRaise = new MotorRaise(arm);
        armLower = new MotorLower(arm);
        armStop = new MotorStop(arm);
        arm.setDefaultCommand(armStop);

        mwrist = new MotorSubsystem(
                hmap,
                "ROTATE",
                1, -1.,0,
                Motor.GoBILDA.RPM_312,
                Motor.ZeroPowerBehavior.BRAKE
        );
        mwristRaise = new MotorRaise(mwrist);
        mwristLower = new MotorLower(mwrist);
        mwristStop = new MotorStop(mwrist);
        mwrist.setDefaultCommand(mwristStop);


        //Intake
        intake = new CRServoSubsystem(
                hmap,
                "GRIPPER",
                1.0,
                -1.0
        );
        intake2 = new CRServoSubsystem(
                hmap,
                "GRIPPER2",
                -1.0,
                1.0
        );
        intakeForward = new CRServoForwardDual(intake, intake2);
        intakeBackward = new CRServoBackwardDual(intake, intake2);
        intakeStop = new CRServoStopDual(intake, intake2);
        intake.setDefaultCommand(intakeStop);
        intake2.setDefaultCommand(intakeStop);

        //Wrist
        wrist = new ServoSubsystem(hmap, "WRIST");
        wristCenter = new ServoSetPosition(wrist, 0.1);
        wristScore = new ServoSetPosition(wrist, .45);




        //DistanceSensor
        //dist1 = new DistanceSensorSubsystem(hmap, "DISTANCE");


    }
    //private RevBlinkinLedDriver.BlinkinPattern getAlliance(String alliance){
      //  if (alliance == "red") {
        //    return RevBlinkinLedDriver.BlinkinPattern.RED;
        //} else if (alliance == "blue") {
          //  return RevBlinkinLedDriver.BlinkinPattern.BLUE;
        //} else {
          //  return RevBlinkinLedDriver.BlinkinPattern.GRAY;
        //}
    //}
    //private RevBlinkinLedDriver.BlinkinPattern getChase(String alliance){
      //  if (alliance == "red") {
        //    return RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
        //} else if (alliance == "blue") {
          //  return RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
        //} else {
          //  return RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY;
        //}
    //}
}
