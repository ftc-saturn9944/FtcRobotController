package org.firstinspires.ftc.teamcode;

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

        } else if(zone == 2){

        } else {

        }

    }

    public boolean isFinished(){
        return true;
    }

}
