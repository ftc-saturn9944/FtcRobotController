package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBench {

    public RevIMU imu;
    public DigitalLEDSubsystem d1;
    public DistanceSensorSubsystem dist1;
    public ColorSensorSubsystem color1;
    public TouchSensorSubsystem touch1;
    public WebcamSubsystem cam1;


    public TestBench(
            HardwareMap hmap
    ) {

        // DigitalLED
        d1 = new  DigitalLEDSubsystem(hmap, "GREEN1", "RED1");

        // Drive
        imu = new RevIMU(hmap);
        imu.init();

        // Distance Sensor
        // I2C
        dist1 = new DistanceSensorSubsystem(hmap, "DIST1");

        // Color Sensor
        // I2C
        color1 = new ColorSensorSubsystem(hmap, "COLOR1");

        // Touch Sensor
        // Digital
        touch1 = new TouchSensorSubsystem(hmap, "TOUCH1");

        // Webcam
        cam1 = new WebcamSubsystem(hmap, "CAM1");



    }
}
