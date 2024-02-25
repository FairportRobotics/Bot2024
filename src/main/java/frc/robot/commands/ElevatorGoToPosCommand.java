package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class ElevatorGoToPosCommand extends Command {

    public enum ElevatorPosition {
        kHome,
        kAMP,
    }

    private ScoringSubsystem _scoringSubsystem;
    private ElevatorPosition requestPos;

    private StatusSignal<Double> leftPosition;
    private StatusSignal<Double> rightPosition;

    final PositionVoltage rightPositionRequest;
    final PositionVoltage leftPositionRequest;

    public ElevatorGoToPosCommand(ScoringSubsystem scoringSubsystem, ElevatorPosition pos) {
        _scoringSubsystem = scoringSubsystem;
        addRequirements(_scoringSubsystem);
        requestPos = pos;

        rightPosition = _scoringSubsystem.elevatorRightMotor.getPosition();
        leftPosition = _scoringSubsystem.elevatorLeftMotor.getPosition();

        rightPositionRequest = new PositionVoltage(0).withSlot(0);
        leftPositionRequest = new PositionVoltage(0).withSlot(0);
    }

    @Override
    public void initialize() {

        if (requestPos == ElevatorPosition.kHome) {
            _scoringSubsystem.elevatorRightMotor.setControl(rightPositionRequest.withPosition(_scoringSubsystem.rightHomePos));
            _scoringSubsystem.elevatorLeftMotor.setControl(leftPositionRequest.withPosition(_scoringSubsystem.leftHomePos));
        } else if (requestPos == ElevatorPosition.kAMP) {
            _scoringSubsystem.elevatorRightMotor.setControl(rightPositionRequest.withPosition(_scoringSubsystem.rightHomePos + 9));
            _scoringSubsystem.elevatorRightMotor.setControl(leftPositionRequest.withPosition(_scoringSubsystem.leftHomePos + 9));
        }
    }

    @Override
    public boolean isFinished() {
        if (requestPos == ElevatorPosition.kHome) {
            return leftPosition.getValue() == leftPositionRequest.Position &&
                rightPosition.getValue() == rightPositionRequest.Position;
        } else if (requestPos == ElevatorPosition.kAMP) {
            return leftPosition.getValue() == leftPositionRequest.Position &&
                rightPosition.getValue() == rightPositionRequest.Position;
        }

        return true;
    }

}
