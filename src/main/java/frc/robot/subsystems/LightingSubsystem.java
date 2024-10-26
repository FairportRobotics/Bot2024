package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

//import org.apache.commons.lang3.ObjectUtils.Null;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightingSubsystem extends SubsystemBase {

    public static CANdle _CANdle;
    String currentColor = "";

    public LightingSubsystem() 
    {
       _CANdle = new CANdle(0);
       CANdleConfiguration config = new CANdleConfiguration();
       config.stripType = LEDStripType.BRG;
       _CANdle.configAllSettings(config);
    }

    public void setColor(int r, int g, int b, double a) 
    {
        _CANdle.clearAnimation(0);
        _CANdle.setLEDs(r, g, b);
        _CANdle.configBrightnessScalar(a);
    }

    public void fire()
    {
        FireAnimation animation = new FireAnimation();
        animation.setLedOffset(8);
        animation.setSpeed(0.1);
        animation.setSparking(0.1);
        _CANdle.animate(animation);
    }

    public void rainbow()
    {
        RainbowAnimation animation = new RainbowAnimation();
        animation.setLedOffset(8);
        animation.setSpeed(0.5);
        _CANdle.animate(animation);
    }

    public void larson(){
        LarsonAnimation animation = new LarsonAnimation(255, 0, 0);
        animation.setSpeed(0.01);
        animation.setNumLed(110);
        animation.setBounceMode(BounceMode.Center);
        _CANdle.animate(animation);
    }

    public void cachow()
    {
        _CANdle.setLEDs(195, 51, 50);
    }

    public void setClimbColor()
    {
        _CANdle.setLEDs(255, 215, 0);
    }
}
