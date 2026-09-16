package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotSetupTesting {
    public RevIMU imu;
    public static boolean FIELD_CENTRIC = false;
    public MecanumSubsystem drive;
    public MotorEx frontLeft, frontRight, backLeft, backRight;

    public RobotSetupTesting(HardwareMap hmap
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

     /*   drive = new MecanumSubsystem(
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

      */
    }
}
