package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class ElevatorGoToPosCommand extends Command {

    public enum ElevatorPosition {
        kHome,
        kAMP,
    }

    private ScoringSubsystem _scoringSubsystem;
    private ElevatorPosition requestPos;

    public ElevatorGoToPosCommand(ScoringSubsystem scoringSubsystem, ElevatorPosition pos) {
        _scoringSubsystem = scoringSubsystem;
        addRequirements(_scoringSubsystem);
        requestPos = pos;
    }

    @Override
    public void initialize() {

        if(_scoringSubsystem.leftHomePos == -1.1111 || _scoringSubsystem.rightHomePos == -1.1111){
            beforeStarting(new ElevatorAutoHomeCommand(_scoringSubsystem));
        } 

        if (requestPos == ElevatorPosition.kHome) {
            _scoringSubsystem.elevatorRightMotor.setPosition(_scoringSubsystem.rightHomePos);
            _scoringSubsystem.elevatorLeftMotor.setPosition(_scoringSubsystem.leftHomePos);
        } else if (requestPos == ElevatorPosition.kAMP) {
            _scoringSubsystem.elevatorRightMotor.setPosition(_scoringSubsystem.rightHomePos + 9);
            _scoringSubsystem.elevatorRightMotor.setPosition(_scoringSubsystem.leftHomePos + 9);
        }
    }

    @Override
    public boolean isFinished() {
        if (requestPos == ElevatorPosition.kHome) {
            return _scoringSubsystem.elevatorLeftMotor.getPosition().getValueAsDouble() == _scoringSubsystem.leftHomePos &&
                _scoringSubsystem.elevatorRightMotor.getPosition().getValueAsDouble() == _scoringSubsystem.rightHomePos;
        } else if (requestPos == ElevatorPosition.kAMP) {
            return _scoringSubsystem.elevatorLeftMotor.getPosition().getValueAsDouble() == _scoringSubsystem.leftHomePos + 9 &&
                _scoringSubsystem.elevatorRightMotor.getPosition().getValueAsDouble() == _scoringSubsystem.rightHomePos + 9;
        }

        return true;
    }

}
