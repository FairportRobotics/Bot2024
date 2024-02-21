
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class FeederOnCommand extends Command {

    IntakeSubsystem _intakeSubsystem;
    double speed;

    public FeederOnCommand(IntakeSubsystem intakeSubsystem, double speed) {
        _intakeSubsystem = intakeSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        _intakeSubsystem.feederMotor.set(speed);
    }
}
