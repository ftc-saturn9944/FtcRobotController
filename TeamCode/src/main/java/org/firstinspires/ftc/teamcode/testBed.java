package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@TeleOp(name = "TestBed", group = "LinearOpMode")
public class testBed extends LinearOpMode {
    // * Hardware Init

    private ElapsedTime runtime = new ElapsedTime();
    // ** Motors
    public DcMotorEx testMotor = null;

    // ** Servos
    public Servo linear1 = null;
    public Servo carousel1 = null;

    // ** I2C
    public RevColorSensorV3 color1;
    public Rev2mDistanceSensor dist1;

    // ** Digital
    public RevTouchSensor touch1;
    public RevTouchSensor touch2;

    // ** Odometry
    public BNO055IMU imu;

    // ** Analog


    @Override
    public void runOpMode() throws InterruptedException {
        testMotor = hardwareMap.get(DcMotorEx.class, "MOTOR1");
        linear1 = hardwareMap.get(Servo.class, "Linear");
        carousel1 = hardwareMap.get(Servo.class, "Rotation");

        color1 = hardwareMap.get(RevColorSensorV3.class, "ColorV3");
        dist1 = hardwareMap.get(Rev2mDistanceSensor.class, "Distance1");

        touch1 = hardwareMap.get(RevTouchSensor.class, "Touch1");
        touch2 = hardwareMap.get(RevTouchSensor.class, "Touch2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ArrayList<Double> distCalc = new ArrayList<Double>();
        Integer DIST_COUNT = 5;
        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            double dist = dist1.getDistance(DistanceUnit.INCH);

            if (distCalc.size() == DIST_COUNT) {
                distCalc.remove(0);
            }
            distCalc.add(dist);
            double distAvg = distCalc.stream()
                    .mapToDouble(a -> a)
                    .sum() / DIST_COUNT;

            telemetry.addData("Dist1 in:", dist);
            telemetry.addData("Dist1 in avg", distAvg);
            telemetry.addData("ColorDist in:" , color1.getDistance(DistanceUnit.INCH));
            telemetry.addData("ColorR:" , color1.red());
            telemetry.addData("ColorG:", color1.green());
            telemetry.addData("ColorB:", color1.blue());
            telemetry.addData("ColorA:", color1.alpha());
            telemetry.addData("Color argb:", color1.argb());
            telemetry.addData("Touch1:", touch1.isPressed());
            telemetry.addData("Touch2:", touch2.isPressed());
            telemetry.update();
        }
    }
}
