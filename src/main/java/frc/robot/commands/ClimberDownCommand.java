package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDownCommand extends Command{

    ClimberSubsystem _climberSubsystem;
    double speed;

    public ClimberDownCommand(ClimberSubsystem climberSubsystem, double speed){
        _climberSubsystem = climberSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        _climberSubsystem.climberLeftMotor.set(-speed);
        _climberSubsystem.climberRightMotor.set(-speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
