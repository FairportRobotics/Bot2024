package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class ShooterOffCommand extends Command{

    ScoringSubsystem _scoringSubsystem;

    public ShooterOffCommand(ScoringSubsystem scoringSubsystem){
        _scoringSubsystem = scoringSubsystem;
        addRequirements(_scoringSubsystem);
    }

    @Override
    public void initialize() {
        _scoringSubsystem.shooterLeftMotor.set(0);
        _scoringSubsystem.shooterRightMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
