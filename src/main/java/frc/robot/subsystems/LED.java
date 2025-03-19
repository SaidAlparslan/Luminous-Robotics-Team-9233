package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final DigitalOutput redLED;
    private final DigitalOutput greenLED;
    private final DigitalOutput blueLED;
    private final Timer blinkTimer = new Timer();
    private boolean isBlinkingred = false;
    private boolean isBlinkingblue = false;

    public LED(int redPort, int greenPort, int bluePort) {
        redLED = new DigitalOutput(redPort);
        greenLED = new DigitalOutput(greenPort);
        blueLED = new DigitalOutput(bluePort);
        turnOff();
        blinkTimer.start();
    }

    public void setColor(boolean red, boolean green, boolean blue) {
        redLED.set(red);
        greenLED.set(green);
        blueLED.set(blue);
        
    }

    public boolean isBlueOn(){
        return blueLED.get();
    }

    public void setRed() {
        isBlinkingred = false;
        isBlinkingblue = false;

        setColor(true, false, false);
    }

    public void setGreen() {
        isBlinkingred = false;
        isBlinkingblue = false;

        setColor(false, true, false);
    }

    public void blinkBlue() {
        isBlinkingred = false;
        isBlinkingblue = true;
    }

    public void blinkRed() {
        isBlinkingred = true;
        isBlinkingblue = false;

    }

    public void turnOff() {
        isBlinkingred = false;
        isBlinkingblue = false;

        setColor(false, false, false);
    }

    @Override
    public void periodic() {
        if (isBlinkingblue) {
            boolean blinkstateblue = ((int) (blinkTimer.get() * 4) % 2 == 0);
            setColor(false, false, blinkstateblue);
        }
        else if (isBlinkingred) {
            boolean blinkStatered = ((int) (blinkTimer.get() * 4) % 2 == 0);
            setColor(blinkStatered, false, false);
        }
        SmartDashboard.putBoolean("LED-RED: ", redLED.get());
        SmartDashboard.putBoolean("LED-GREEN: ", greenLED.get());
        SmartDashboard.putBoolean("LED-BLUE: ", blueLED.get());
    }
}
