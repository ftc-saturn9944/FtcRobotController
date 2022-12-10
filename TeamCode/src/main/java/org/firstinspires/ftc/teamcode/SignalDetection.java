package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class SignalDetection extends CommandBase {
    private final CameraSubsystem camera;

    public SignalDetection (CameraSubsystem subsystem){
        camera = subsystem;
        addRequirements(camera);
    }

    public void initialize(){
        camera.initializeCamera();
    }

    public void execute() {
        camera.detectColor();
    }

    public boolean isFinished() {
        return (camera.getColor() != "park");
    }
}

