package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class ElevatorDownCommand extends Command{

    ScoringSubsystem _scoringSubsystem;
    double speed;

    public ElevatorDownCommand(ScoringSubsystem scoringSubsystem, double speed){
        _scoringSubsystem = scoringSubsystem;
        this.speed = speed;
        addRequirements(_scoringSubsystem);
    }

    @Override
    public void initialize() {
        _scoringSubsystem.elevatorLeftMotor.set(-speed);
        _scoringSubsystem.elevatorRightMotor.set(-speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
