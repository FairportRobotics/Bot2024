package frc.robot.commands;

import com.ctre.phoenix6.StatusSignal;

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
        _scoringSubsystem.elevatorLeftMotor.set(-0.1);
        _scoringSubsystem.elevatorRightMotor.set(-0.1);
    }

    @Override
    public boolean isFinished() {
        return !_scoringSubsystem.bottomlimitSwitch.get();
    }

    @Override
    public void end(boolean interrupted) {
        _scoringSubsystem.elevatorLeftMotor.set(0.0);
        _scoringSubsystem.elevatorRightMotor.set(0.0);

        StatusSignal<Double> leftPos = _scoringSubsystem.elevatorLeftMotor.getPosition();
        StatusSignal<Double> rightPos = _scoringSubsystem.elevatorRightMotor.getPosition();

        leftPos.waitForUpdate(1.0);
        rightPos.waitForUpdate(1.0);

        _scoringSubsystem.leftHomePos = leftPos.getValue();
        _scoringSubsystem.rightHomePos = rightPos.getValue();
    }

}
