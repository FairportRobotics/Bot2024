
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNoteToFeederCommand extends Command {

    IntakeSubsystem _intakeSubsystem;

    VelocityVoltage intakeRequest;
    VelocityVoltage feederRequest;

    public IntakeNoteToFeederCommand(IntakeSubsystem intakeSubsystem) {
        _intakeSubsystem = intakeSubsystem;

        addRequirements(_intakeSubsystem);

        intakeRequest = new VelocityVoltage(0).withSlot(0);
        feederRequest = new VelocityVoltage(0).withSlot(0);
    }

    @Override
    public void initialize() {
        //_intakeSubsystem.feederMotor.setControl(feederRequest.withVelocity(20));
        _intakeSubsystem.feederMotor.set(0.65);

        _intakeSubsystem.intakeLeftMotor.set(0.5);
        // _intakeSubsystem.intakeRightMotor.set(ControlMode.PercentOutput, -1.5);
    }

    @Override
    public boolean isFinished() {
        return !_intakeSubsystem.shooterSensor.get();
    }

    @Override
    public void end(boolean interrupted) {
        _intakeSubsystem.feederMotor.set(0);
        _intakeSubsystem.intakeLeftMotor.set(0);
        // _intakeSubsystem.intakeRightMotor.set(ControlMode.PercentOutput, -1.0);
    }
}
