
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class FeederOffCommand extends Command {

    IntakeSubsystem _intakeSubsystem;
    double speed;

    public FeederOffCommand(IntakeSubsystem intakeSubsystem) {
        _intakeSubsystem = intakeSubsystem;
        addRequirements(_intakeSubsystem);
    }

    @Override
    public void initialize() {
        _intakeSubsystem.feederMotor.set(0.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
