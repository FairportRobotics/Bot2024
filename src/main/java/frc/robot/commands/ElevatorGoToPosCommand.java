package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ScoringSubsystem;

public class ElevatorGoToPosCommand extends Command {

    public enum ElevatorPosition {
        kNone,
        kHome,
        kAMP,
    }

    private ScoringSubsystem _scoringSubsystem;
    private double requestPosRots = Double.MAX_VALUE;

    private StatusSignal<Angle> leftPosition;
    private StatusSignal<Angle> rightPosition;

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
        _scoringSubsystem.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Coast);
        _scoringSubsystem.elevatorRightMotor.setNeutralMode(NeutralModeValue.Coast);

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

            SmartDashboard.putNumber("Ele Left Pos", leftPosition.getValueAsDouble());
            SmartDashboard.putNumber("Ele Right", rightPosition.getValueAsDouble());

            return (Math.abs(leftPosition.getValueAsDouble() - (requestPosRots + _scoringSubsystem.leftHomePos)) <= 0.1 ||
                    Math.abs(rightPosition.getValueAsDouble() - (requestPosRots + _scoringSubsystem.rightHomePos)) <= 0.1);
        }

        return false;

    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.noteAquired = false;
        _scoringSubsystem.elevatorLeftMotor.stopMotor();
        _scoringSubsystem.elevatorRightMotor.stopMotor();
        _scoringSubsystem.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
        _scoringSubsystem.elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

}
