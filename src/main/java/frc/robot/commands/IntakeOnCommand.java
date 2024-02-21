package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOnCommand extends Command{

    IntakeSubsystem _IntakeSubsystem;
    double intakeSpeed;

    public IntakeOnCommand(IntakeSubsystem intakeSubsystem, double intakeSpeed){
        _IntakeSubsystem = intakeSubsystem;
        this.intakeSpeed = intakeSpeed;
    }

    @Override
    public void initialize() {
        _IntakeSubsystem.intakeLeftMotor.set(ControlMode.PercentOutput, intakeSpeed);
        _IntakeSubsystem.intakeRightMotor.set(ControlMode.PercentOutput, intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
