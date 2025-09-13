package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class WebcamSubsystem extends SubsystemBase {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public WebcamSubsystem (HardwareMap hmap, String name){
        initAprilTag(hmap, name);
    }

    private void initAprilTag(HardwareMap hmap, String name) {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hmap.get(WebcamName.class, name),
                aprilTag
        );
    }

    public VisionPortal visionPortal() {
        return visionPortal;
    }

    public List<AprilTagDetection> getAprilTagTelemetry() {
        return aprilTag.getDetections();
    }
}
