package frc.robot.subsystems;

import java.io.Serial;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class Scoring extends SubsystemBase {

    TalonSRX testMoter = new TalonSRX(17);

    TalonFX upMotor = new TalonFX(30);   //Device Id's need to be changed
    TalonFX downMotor = new TalonFX(31);
    TalonFX elevator = new TalonFX(32);
    TalonFX Shoot = new TalonFX(33);

    // TalonSRX testMoter = new TalonSRX(17);
    // AnalogPotentiometer pot = new AnalogPotentiometer(new AnalogInput(0), 2, -1);

    public Scoring() {
    }

    public Command ScoringMethodCommand() {
        // Inline construction of command goes here.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    public boolean ScoringCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    // SHOOTER
    public Command Shootable(double shootforce) {
        return new Command() {
            @Override
            public void execute() {
                Shoot.set(shootforce);
            }
        };
    }
}