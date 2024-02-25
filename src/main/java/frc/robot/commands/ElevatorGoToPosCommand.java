package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private StatusSignal<Double> leftPosError;
    private StatusSignal<Double> rightPosError;

    final PositionVoltage rightPositionRequest;
    final PositionVoltage leftPositionRequest;

    public ElevatorGoToPosCommand(ScoringSubsystem scoringSubsystem, ElevatorPosition pos) {
        _scoringSubsystem = scoringSubsystem;
        addRequirements(_scoringSubsystem);
        requestPos = pos;

        rightPosition = _scoringSubsystem.elevatorRightMotor.getPosition();
        leftPosition = _scoringSubsystem.elevatorLeftMotor.getPosition();

        leftPosError = _scoringSubsystem.elevatorLeftMotor.getClosedLoopError();
        rightPosError = _scoringSubsystem.elevatorRightMotor.getClosedLoopError();

        rightPositionRequest = new PositionVoltage(0).withSlot(0);
        leftPositionRequest = new PositionVoltage(0).withSlot(0);
    }

    @Override
    public void initialize() {

        if (requestPos == ElevatorPosition.kHome) {
            _scoringSubsystem.elevatorRightMotor
                    .setControl(rightPositionRequest.withPosition(_scoringSubsystem.rightHomePos));
            _scoringSubsystem.elevatorLeftMotor
                    .setControl(leftPositionRequest.withPosition(_scoringSubsystem.leftHomePos));
        } else if (requestPos == ElevatorPosition.kAMP) {
            _scoringSubsystem.elevatorRightMotor
                    .setControl(rightPositionRequest.withPosition(_scoringSubsystem.rightHomePos + 9));
            _scoringSubsystem.elevatorRightMotor
                    .setControl(leftPositionRequest.withPosition(_scoringSubsystem.leftHomePos + 9));
        }
    }

    @Override
    public void execute() {

        leftPosError.waitForUpdate(0.1);
        rightPosError.waitForUpdate(0.1);

        SmartDashboard.putNumber("Elevator Left Error", leftPosError.getValue());
        SmartDashboard.putNumber("Elevator Right Error", rightPosError.getValue());
    }

    @Override
    public boolean isFinished() {
        leftPosError.waitForUpdate(0.1);
        rightPosError.waitForUpdate(0.1);

        if (requestPos == ElevatorPosition.kHome) {
            return !_scoringSubsystem.bottomlimitSwitch.get();
        } else {
            return (Math.abs(leftPosError.getValue()) <= 0.00001 &&
                    Math.abs(rightPosError.getValue()) <= 0.00001);
        }

    }

    @Override
    public void end(boolean interrupted) {
        _scoringSubsystem.elevatorLeftMotor.stopMotor();
        _scoringSubsystem.elevatorRightMotor.stopMotor();
    }

}
