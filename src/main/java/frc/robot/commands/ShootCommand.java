package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;

public class ShootCommand extends SequentialCommandGroup {
    ScoringSubsystem scoringSubsystem;
    IntakeSubsystem intakeSubsystem;

    public ShootCommand(ScoringSubsystem scoringSubsystem, IntakeSubsystem intakeSubsystem) {
        this.scoringSubsystem = scoringSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        // addRequirements(scoringSubsystem);

        addCommands(Commands.parallel(new ShooterOnCommand(scoringSubsystem, 85)),
                new FeederOnCommand(intakeSubsystem, 1.0),
                new WaitCommand(0.8),
                new ShooterOffCommand(scoringSubsystem),
                new FeederOffCommand(intakeSubsystem));
    }
}
