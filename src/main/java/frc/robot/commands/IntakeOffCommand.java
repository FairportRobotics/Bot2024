package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOffCommand extends Command{

    IntakeSubsystem _IntakeSubsystem;

    public IntakeOffCommand(IntakeSubsystem intakeSubsystem){
        _IntakeSubsystem = intakeSubsystem;
    }

    @Override
    public void initialize() {
        _IntakeSubsystem.intakeLeftMotor.set(ControlMode.PercentOutput, 0.0);
        _IntakeSubsystem.intakeRightMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
