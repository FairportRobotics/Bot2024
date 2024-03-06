package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ScoringSubsystem;

public class ElevatorGoToPosCommand extends Command {

    public enum ElevatorPosition {
        kNone,
        kHome,
        kAMP,
    }

    private ScoringSubsystem _scoringSubsystem;
    private double requestPosRots = Double.MAX_VALUE;

    private StatusSignal<Double> leftPosition;
    private StatusSignal<Double> rightPosition;

    private StatusSignal<Double> leftPosError;
    private StatusSignal<Double> rightPosError;

    final PositionVoltage rightPositionRequest;
    final PositionVoltage leftPositionRequest;

    public ElevatorGoToPosCommand(ScoringSubsystem scoringSubsystem, ElevatorPosition pos) {
        _scoringSubsystem = scoringSubsystem;
        addRequirements(_scoringSubsystem);

        if (pos == ElevatorPosition.kAMP) {
            requestPosRots = 10.25;
        } else {
            requestPosRots = 0;
        }

        rightPosition = _scoringSubsystem.elevatorRightMotor.getPosition();
        leftPosition = _scoringSubsystem.elevatorLeftMotor.getPosition();

        leftPosError = _scoringSubsystem.elevatorLeftMotor.getClosedLoopError();
        rightPosError = _scoringSubsystem.elevatorRightMotor.getClosedLoopError();

        rightPositionRequest = new PositionVoltage(0).withSlot(0);
        leftPositionRequest = new PositionVoltage(0).withSlot(0);
    }

    public ElevatorGoToPosCommand(ScoringSubsystem scoringSubsystem, double pos) {
        _scoringSubsystem = scoringSubsystem;
        addRequirements(_scoringSubsystem);
        requestPosRots = pos;

        rightPosition = _scoringSubsystem.elevatorRightMotor.getPosition();
        leftPosition = _scoringSubsystem.elevatorLeftMotor.getPosition();

        leftPosError = _scoringSubsystem.elevatorLeftMotor.getClosedLoopError();
        rightPosError = _scoringSubsystem.elevatorRightMotor.getClosedLoopError();

        rightPositionRequest = new PositionVoltage(0).withSlot(0);
        leftPositionRequest = new PositionVoltage(0).withSlot(0);
    }

    @Override
    public void initialize() {

        _scoringSubsystem.elevatorLeftMotor
                .setControl(leftPositionRequest.withPosition(_scoringSubsystem.leftHomePos + requestPosRots));
        _scoringSubsystem.elevatorRightMotor
                .setControl(rightPositionRequest.withPosition(_scoringSubsystem.rightHomePos + requestPosRots));
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {

        leftPosError.refresh();
        rightPosError.refresh();

        if (requestPosRots <= 0) {
            return !_scoringSubsystem.bottomlimitSwitch.get();
        } else if (leftPosition.hasUpdated() && rightPosition.hasUpdated()) {

            SmartDashboard.putNumber("Ele Left Pos", leftPosition.getValue());
            SmartDashboard.putNumber("Ele Right", rightPosition.getValue());

            return (Math.abs(leftPosition.getValue() - (requestPosRots + _scoringSubsystem.leftHomePos)) <= 0.1 ||
                    Math.abs(rightPosition.getValue() - (requestPosRots + _scoringSubsystem.rightHomePos)) <= 0.1);
        }

        return false;

    }

    @Override
    public void end(boolean interrupted) {
        _scoringSubsystem.elevatorLeftMotor.stopMotor();
        _scoringSubsystem.elevatorRightMotor.stopMotor();
    }

}
