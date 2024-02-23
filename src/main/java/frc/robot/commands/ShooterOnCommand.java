package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class ShooterOnCommand extends Command{

    ScoringSubsystem _scoringSubsystem;
    double speed;

    public ShooterOnCommand(ScoringSubsystem scoringSubsystem, double speed){
        _scoringSubsystem = scoringSubsystem;
        addRequirements(_scoringSubsystem);
        this.speed = speed;
    }

    @Override
    public void initialize() {
        _scoringSubsystem.shooterTopMotor.set(speed);
        _scoringSubsystem.shooterBottomMotor.set(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
