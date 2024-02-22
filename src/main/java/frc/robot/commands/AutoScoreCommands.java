package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;

public class AutoScoreCommands {

    ScoringSubsystem _scoringSubsystem;
    ClimberSubsystem _climberSubsystem;
    IntakeSubsystem _intakeSubsystem;

    public AutoScoreCommands(ScoringSubsystem scoringSubsystem, ClimberSubsystem climberSubsystem,
            IntakeSubsystem intakeSubsystem) {
        _scoringSubsystem = scoringSubsystem;
        _climberSubsystem = climberSubsystem;
        _intakeSubsystem = intakeSubsystem;
    }

    public Command scoreSpeakerCommand = Commands.sequence(
            new IntakeNoteToFeederCommand(_intakeSubsystem),
            //TODO: GOTO SPEAKER ZONE
            Commands.parallel(new ShooterOnCommand(_scoringSubsystem, 1.0),
                    new FeederOnCommand(_intakeSubsystem, 0.15)),
            new WaitCommand(1.0),
            Commands.parallel(new ShooterOffCommand(_scoringSubsystem),
                    new FeederOffCommand(_intakeSubsystem)));

    public Command getScoreSpeakerCommand() {
        return scoreSpeakerCommand;
    }

}
