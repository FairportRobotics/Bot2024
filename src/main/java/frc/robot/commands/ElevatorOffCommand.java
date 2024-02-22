package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class ElevatorOffCommand extends Command{

    ScoringSubsystem _scoringSubsystem;

    public ElevatorOffCommand(ScoringSubsystem scoringSubsystem){
        _scoringSubsystem = scoringSubsystem;
        addRequirements(_scoringSubsystem);
    }

    @Override
    public void initialize() {
        _scoringSubsystem.elevatorLeftMotor.set(0);
        _scoringSubsystem.elevatorRightMotor.set(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
