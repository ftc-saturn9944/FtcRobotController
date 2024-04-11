package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import java.util.concurrent.TimeUnit;

public class ProximitySensor extends CommandBase {
    private LEDSubsystem led;
    private DistanceSensorSubsystem distance;
    private Timing.Timer timer;
    private double near, scoring;
    private RevBlinkinLedDriver.BlinkinPattern farLED, nearLED, scoringLED;
    private enum Proximity {
        Far, Near, Scoring
    }
    private Proximity prox;
    public ProximitySensor (
            LEDSubsystem led,
            DistanceSensorSubsystem distance,
            double near,
            double scoring
    ) {
        this.led = led;
        this.distance = distance;
        this.near = near;
        this.scoring = scoring;
        this.timer = new Timing.Timer(1, TimeUnit.SECONDS);
        farLED = RevBlinkinLedDriver.BlinkinPattern.GRAY;
        nearLED = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        scoringLED = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        addRequirements(led, distance);
    }
    public void initialize (){
        this.prox = Proximity.Far;
        led.setLed(farLED);
        timer.start();
    }
    public void execute(){
        if (timer.done()){
            if (this.prox==Proximity.Far){
                if (distance.getDistance()<scoring){
                    
                } else if (distance.getDistance()<near) {
                    
                }
            } else if (this.prox==Proximity.Near) {
                if (distance.getDistance()<scoring){
                    
                } else if (distance.getDistance()>near) {
                    
                }
            }else {
                if (distance.getDistance()>near) {
                    
                } else if (distance.getDistance()>scoring) {
                    
                }
            }
        }
    }
}
