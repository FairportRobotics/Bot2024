package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOffCommand extends Command{

    IntakeSubsystem _IntakeSubsystem;

    public IntakeOffCommand(IntakeSubsystem intakeSubsystem){
        _IntakeSubsystem = intakeSubsystem;

        addRequirements(_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        _IntakeSubsystem.intakeLeftMotor.stopMotor();
//        _IntakeSubsystem.intakeRightMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
