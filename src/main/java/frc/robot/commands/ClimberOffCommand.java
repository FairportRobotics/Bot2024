package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberOffCommand extends Command{

    ClimberSubsystem _climberSubsystem;

    public ClimberOffCommand(ClimberSubsystem climberSubsystem){
        _climberSubsystem = climberSubsystem;
    }

    @Override
    public void initialize() {
        _climberSubsystem.climberLeftMotor.set(0);
        _climberSubsystem.climberRightMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
