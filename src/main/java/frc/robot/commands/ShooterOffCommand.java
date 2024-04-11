package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ScoringSubsystem;

public class ShooterOffCommand extends Command{

    ScoringSubsystem _scoringSubsystem;

    public ShooterOffCommand(ScoringSubsystem scoringSubsystem){
        _scoringSubsystem = scoringSubsystem;
        addRequirements(_scoringSubsystem);
    }

    @Override
    public void initialize() {
        _scoringSubsystem.shooterTopMotor.set(0);
        _scoringSubsystem.shooterBottomMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted)
    {
        RobotContainer.noteAquired = false;
    }
}
