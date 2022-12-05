package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.CommandBase;
import android.graphics.Bitmap;

public class AutoPark extends CommandBase {

    private final CameraSubsystem camera;

    public AutoPark(CameraSubsystem subsystem){
        camera = subsystem;
        addRequirements(camera);
    }

    public void initialize(){
        Bitmap bmp = camera.capture();
        int zone = camera.getParking(bmp);

        if(zone == 1){
            telemetry.addData("Zone:", " 1");
        } else if(zone == 2){
            telemetry.addData("Zone:", " 2");
        } else {
            telemetry.addData("Zone:", " 3");
        }

    }

    public boolean isFinished(){
        return true;
    }

}
