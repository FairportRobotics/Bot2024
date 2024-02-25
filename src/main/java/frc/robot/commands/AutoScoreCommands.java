package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ElevatorGoToPosCommand.ElevatorPosition;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;

public class AutoScoreCommands {

        ScoringSubsystem _scoringSubsystem;
        ClimberSubsystem _climberSubsystem;
        IntakeSubsystem _intakeSubsystem;
        CommandSwerveDrivetrain _commandSwerveDrivetrain;

        Pose2d _roboSpeakerPose2d;
        Pose2d _roboAmpPose2d;
        PathConstraints _constraints;

        public Command scoreSpeakerCommand;

        public Command scoreAmpCommand;

        public AutoScoreCommands(ScoringSubsystem scoringSubsystem, ClimberSubsystem climberSubsystem,
                        IntakeSubsystem intakeSubsystem, CommandSwerveDrivetrain commandSwerveDrivetrain) {
                _scoringSubsystem = scoringSubsystem;
                _climberSubsystem = climberSubsystem;
                _intakeSubsystem = intakeSubsystem;
                _commandSwerveDrivetrain = commandSwerveDrivetrain;
                _roboSpeakerPose2d = commandSwerveDrivetrain.roboSpeakerPose2d;
                _roboAmpPose2d = commandSwerveDrivetrain.roboAmpPose2d;
                _constraints = commandSwerveDrivetrain.constraints;

                scoreSpeakerCommand = Commands.sequence(
                                Commands.parallel(new IntakeNoteToFeederCommand(_intakeSubsystem),
                                                new ElevatorGoToPosCommand(scoringSubsystem, ElevatorPosition.kHome),
                                                AutoBuilder.pathfindToPose(
                                                                _roboSpeakerPose2d,
                                                                _constraints,
                                                                0.0, // Goal end velocity in meters/sec
                                                                0.0 // Rotation delay distance in meters. This is how
                                                                    // far the robot should travel
                                                                    // before attempting to rotate.
                                                )),
                                new ShooterOnCommand(_scoringSubsystem, 100),
                                new FeederOnCommand(_intakeSubsystem, 0.15),
                                new WaitCommand(1.0),
                                Commands.parallel(new ShooterOffCommand(_scoringSubsystem),
                                                new FeederOffCommand(_intakeSubsystem)));

                scoreAmpCommand = Commands.sequence(
                                Commands.parallel(new IntakeNoteToFeederCommand(intakeSubsystem),
                                                new ElevatorGoToPosCommand(scoringSubsystem, ElevatorPosition.kHome),
                                                AutoBuilder.pathfindToPose(
                                                                _roboAmpPose2d,
                                                                _constraints,
                                                                0.0, // Goal end velocity in meters/sec
                                                                0.0 // Rotation delay distance in meters. This is how
                                                                    // far the robot should travel
                                                                    // before attempting to rotate.
                                                )),
                                new ElevatorGoToPosCommand(scoringSubsystem, ElevatorPosition.kAMP),
                                new FeederOnCommand(intakeSubsystem, -1.0),
                                new WaitCommand(0.5),
                                new ElevatorGoToPosCommand(scoringSubsystem, ElevatorPosition.kHome));

        }

}
