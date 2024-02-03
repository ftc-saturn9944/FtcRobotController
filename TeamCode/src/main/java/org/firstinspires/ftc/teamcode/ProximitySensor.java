package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import java.util.concurrent.TimeUnit;

public class ProximitySensor extends CommandBase {
    private LEDSubsystem led;
    private DistanceSubsystem distance;

    private Timing.Timer timer;

    private double near, scoring;
    private RevBlinkinLedDriver.BlinkinPattern farLED = RevBlinkinLedDriver.BlinkinPattern.GRAY;
    private RevBlinkinLedDriver.BlinkinPattern nearLED = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    private RevBlinkinLedDriver.BlinkinPattern scoringLED = RevBlinkinLedDriver.BlinkinPattern.GREEN;

    private RevBlinkinLedDriver.BlinkinPattern allianceColor;

    private enum Proximity {
        Far,
        Near,
        Scoring
    }
    private Proximity prox;
    public ProximitySensor (
            LEDSubsystem led,
            DistanceSubsystem distance,
            double near,
            double scoring,
            RevBlinkinLedDriver.BlinkinPattern allianceColor
    ) {
        this.led = led;
        this.distance = distance;
        timer = new Timing.Timer(1, TimeUnit.SECONDS);
        this.prox = Proximity.Far;
        this.near = near;
        this.scoring = scoring;
        this.allianceColor = allianceColor;
        //this.farLED = this.allianceColor;
        addRequirements(this.led, this.distance);
    }

    public void initialize() {
        this.prox = Proximity.Far;
        led.setLed(farLED);
        timer.start();
    }

    @Override
    public void execute() {
        // Wait for timer to finish to ensure only every second
        // First attempt didn't work
        if (timer.done()) {
            if (this.prox == Proximity.Far) {
                // Currently far away
                if (distance.getDistance() < scoring) {
                    // Scoring higher priority than near
                    led.setLed(scoringLED);
                    this.prox = Proximity.Scoring;
                    timer.start();
                } else if (distance.getDistance() < near) {
                    // Near if appropriate
                    led.setLed(nearLED);
                    this.prox = Proximity.Near;
                    timer.start();
                }
            } else if (this.prox == Proximity.Near) {
                // Currently near
                if (distance.getDistance() < scoring) {
                    // If scoring then score
                    led.setLed(scoringLED);
                    this.prox = Proximity.Scoring;
                    timer.start();
                } else if (distance.getDistance() > near) {
                    // If outside the near target
                    led.setLed(farLED);
                    this.prox = Proximity.Far;
                    timer.start();
                }
            } else {
                // Not far or near so Proximity.Scoring
                if (distance.getDistance() > near) {
                    // Farther away than near
                    // Far has higher priority than near
                    led.setLed(farLED);
                    this.prox = Proximity.Far;
                    timer.start();
                } else if (distance.getDistance() > scoring) {
                    led.setLed(nearLED);
                    this.prox = Proximity.Near;
                    timer.start();
                }
            }
            // If none apply, don't restart timer.
        }
        /* // Works
        if (timer.done()) {
            if (distance.getDistance() < scoring) {
                led.setLed(scoringLED);
            } else if (distance.getDistance() < near) {
                led.setLed(nearLED);
            } else if (distance.getDistance() > near) {
                led.setLed(farLED);
            }
            timer.start();
        }
         */
    }

    public void end() {
        led.setLed(allianceColor);
    }
}
