package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class Scoring extends SubsystemBase{

    TalonSRX Christ = new TalonSRX(17);
    CommandPS4Controller controller;

    public Scoring(CommandPS4Controller controller) {
        this.controller = controller;
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command ScoringMethodCommand() 
    {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
            /* one-time action goes here */
            });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean ScoringCondition() 
    {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() 
    {
        Christ.set(ControlMode.PercentOutput, controller.getLeftY()); 
    }

    @Override
    public void simulationPeriodic() 
    {
        // This method will be called once per scheduler run during simulation
    }
}
