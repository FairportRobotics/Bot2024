package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class ElevatorAutoHomeCommand extends Command{

    private ScoringSubsystem _scoringSubsystem;
    
    public ElevatorAutoHomeCommand(ScoringSubsystem scoringSubsystem){
        _scoringSubsystem = scoringSubsystem;
        addRequirements(_scoringSubsystem);
    }

    @Override
    public void initialize() {
        _scoringSubsystem.elevatorLeftMotor.set(-0.5);
        _scoringSubsystem.elevatorRightMotor.set(-0.5);
    }

    @Override
    public boolean isFinished() {
        return _scoringSubsystem.bottomlimitSwitch.get();
    }

    @Override
    public void end(boolean interrupted) {
        _scoringSubsystem.elevatorLeftMotor.set(0.0);
        _scoringSubsystem.elevatorRightMotor.set(0.0);

        _scoringSubsystem.leftHomePos = _scoringSubsystem.elevatorLeftMotor.getPosition().getValueAsDouble();
        _scoringSubsystem.rightHomePos = _scoringSubsystem.elevatorRightMotor.getPosition().getValueAsDouble();
    }

}
