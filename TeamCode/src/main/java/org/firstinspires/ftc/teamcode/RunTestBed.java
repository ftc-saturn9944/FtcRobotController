package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class RunTestBed extends CommandOpMode {
    private TestBench robot;

    public void initialize() {
        robot = new TestBench(
                hardwareMap
        );
        telemetry.setAutoClear(false);
        telemetry.update();
        telemetry.addLine("startInitialization");
        telemetry.update();
        robot.cam1.visionPortal().resumeStreaming();



    }
    @Override
    public void run() {
        telemetry.clearAll();
        // telemetry is here
        ArrayList<Integer> colors = robot.color1.getColors();
        List<AprilTagDetection> tags = robot.cam1.getAprilTagTelemetry();
       /* telemetry.addLine("Telemetry:");
        telemetry.addData("Touch Sensor", robot.touch1::getState);
        telemetry.addData("Distance Sensor",robot.dist1::getDistance);
        telemetry.addData("Color Distance", robot.color1::getDistance);
        telemetry.addData("Color Int",robot.color1::getColor);
        telemetry.addData("C1 Alpha", colors.get(0));
        telemetry.addData("C1 Red", colors.get(1));
        telemetry.addData("C1 Blue", colors.get(2));
        telemetry.addData("C1 Green", colors.get(3));
        */
        robot.cam1.visionPortal().resumeStreaming();
        telemetry.addData("State", robot.cam1.visionPortal().getCameraState());
        telemetry.addData("# AprilTags Detected", tags.size());
        for (AprilTagDetection detection : tags) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
        //end telemetry
        telemetry.update();
        super.run();
    }
}
