
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNoteToFeederCommand extends Command {

    IntakeSubsystem _intakeSubsystem;

    public IntakeNoteToFeederCommand(IntakeSubsystem intakeSubsystem) {
        _intakeSubsystem = intakeSubsystem;

        addRequirements(_intakeSubsystem);
    }

    @Override
    public void initialize() {
        _intakeSubsystem.feederMotor.set(0.25);

        _intakeSubsystem.intakeLeftMotor.set(ControlMode.PercentOutput, 1.0);
        _intakeSubsystem.intakeRightMotor.set(ControlMode.PercentOutput, 1.0);
    }

    @Override
    public boolean isFinished() {
        return _intakeSubsystem.feederSensor.get();
    }

    @Override
    public void end(boolean interrupted) {
        _intakeSubsystem.feederMotor.set(0);

        _intakeSubsystem.intakeLeftMotor.set(ControlMode.PercentOutput, 0);
        _intakeSubsystem.intakeRightMotor.set(ControlMode.PercentOutput, 0);
    }
}
