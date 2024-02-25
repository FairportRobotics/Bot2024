// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoScoreCommands;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberOffCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.ElevatorAutoHomeCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorGoToPosCommand;
import frc.robot.commands.ElevatorOffCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.FeederOffCommand;
import frc.robot.commands.FeederOnCommand;
import frc.robot.commands.IntakeNoteToFeederCommand;
import frc.robot.commands.IntakeOffCommand;
import frc.robot.commands.IntakeOnCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterOffCommand;
import frc.robot.commands.ShooterOnCommand;
import frc.robot.commands.ElevatorGoToPosCommand.ElevatorPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController operator = new CommandXboxController(
      Constants.OperatorConstants.kOperatorControllerPort);
  private final CommandXboxController driver = new CommandXboxController(
      Constants.OperatorConstants.kDriverControllerPort); // driver
  private final CommandXboxController systemCheck = new CommandXboxController(
      Constants.OperatorConstants.kSystemCheckControllerPort); // systemCheck

  private final CommandSwerveDrivetrain drivetrainSubsystem = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final ScoringSubsystem scoringSubsystem = new ScoringSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  public SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public class Commands {

    public Command intakeOnCommand = new IntakeOnCommand(intakeSubsystem, 0.7);
    public Command intakeRevCommand = new IntakeOnCommand(intakeSubsystem, -0.7);
    public Command intakeOffCommand = new IntakeOffCommand(intakeSubsystem);
    public Command intakeNoteToFeeder = new IntakeNoteToFeederCommand(intakeSubsystem);

    public Command feederFwdCommand = new FeederOnCommand(intakeSubsystem, 0.25);
    public Command feederFwdCommandSlow = new FeederOnCommand(intakeSubsystem, 0.1);
    public Command feederOffCommand = new FeederOffCommand(intakeSubsystem);
    public Command feederRevCommand = new FeederOnCommand(intakeSubsystem, -0.15);

    public Command shooterOnCommand = new ShooterOnCommand(scoringSubsystem, 200);
    public Command shooterOffCommand = new ShooterOffCommand(scoringSubsystem);

    public Command elevatorUpCommand = new ElevatorUpCommand(scoringSubsystem, 0.1);
    public Command elevatorDownCommand = new ElevatorDownCommand(scoringSubsystem, 0.1);
    public Command elevatorOffCommand = new ElevatorOffCommand(scoringSubsystem);

    public Command elevatorAutoHome = new ElevatorAutoHomeCommand(scoringSubsystem);
    public Command elevatorHomeCommand = new ElevatorGoToPosCommand(scoringSubsystem, ElevatorPosition.kHome);
    public Command elevatorAmpCommand = new ElevatorGoToPosCommand(scoringSubsystem, ElevatorPosition.kAMP);

    public Command climberUpCommand = new ClimberUpCommand(climberSubsystem, 0.40);
    public Command climberDownCommand = new ClimberDownCommand(climberSubsystem, 0.40);
    public Command climberOffCommand = new ClimberOffCommand(climberSubsystem);

    public Command shootCommand = new ShootCommand(scoringSubsystem, intakeSubsystem);

    private AutoScoreCommands autoScoreCommands = new AutoScoreCommands(scoringSubsystem, climberSubsystem,
        intakeSubsystem, drivetrainSubsystem);

    public Command scoreAmpCommand = autoScoreCommands.scoreAmpCommand;
    public Command scoreSpeakerCommand = autoScoreCommands.scoreSpeakerCommand;
  }

  Commands commands;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    final Field2d field;

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
    });

    NamedCommands.registerCommand("Shoot", null);// Register Named Commands

    AutoBuilder.configureHolonomic(
        drivetrainSubsystem::getPose, // Robot pose supplier
        drivetrainSubsystem::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        drivetrainSubsystem::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        drivetrainSubsystem::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                         // Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        drivetrainSubsystem // Reference to this subsystem to set requirements
    );

    commands = new Commands();

    autoChooser.setDefaultOption("None", new Command() {
    });
    SmartDashboard.putData(autoChooser);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // if (bindingChooser.getSelected() && !DriverStation.isFMSAttached()) {
    // SYSTEM CHECK BINDINGS
    // here for Scoring subsystem :) change if too low or high
    systemCheck.x().onTrue(commands.shooterOnCommand);
    systemCheck.x().onFalse(commands.shooterOffCommand);

    systemCheck.rightBumper().onTrue(commands.intakeOnCommand);
    systemCheck.rightBumper().onFalse(commands.intakeOffCommand);
    systemCheck.povDown().onTrue(commands.intakeRevCommand);
    systemCheck.povDown().onFalse(commands.intakeOffCommand);

    // here for intake subsystem :)
    systemCheck.leftBumper().onTrue(commands.feederFwdCommand);
    systemCheck.leftBumper().onFalse(commands.feederOffCommand);
    systemCheck.y().onTrue(commands.feederRevCommand);
    systemCheck.y().onFalse(commands.feederOffCommand);
    // Climber
    systemCheck.rightTrigger().onTrue(commands.climberUpCommand);
    systemCheck.rightTrigger().onFalse(commands.climberOffCommand);
    systemCheck.leftTrigger().onTrue(commands.climberDownCommand);
    systemCheck.leftTrigger().onFalse(commands.climberOffCommand);

    systemCheck.a().onTrue(commands.elevatorAmpCommand);
    systemCheck.b().onTrue(commands.elevatorHomeCommand);
    systemCheck.back().onTrue(commands.elevatorAutoHome);

    systemCheck.povLeft().onTrue(commands.shootCommand);
    systemCheck.povUp().onTrue(commands.intakeNoteToFeeder);

    // operator.a().onTrue(commands.autoScoreCommands.scoreSpeakerCommand);
    // operator.povRight().onTrue(commands.autoScoreCommands.scoreAmpCommand);

    // NORMAL DRIVE MODE BINDINGS
    operator.povUp().onTrue(commands.feederFwdCommandSlow);
    operator.povUp().onFalse(commands.feederOffCommand);

    // NORMAL DRIVE MODE BINDINGS

    driver.a().onTrue(commands.autoScoreCommands.scoreAmpCommand);
    driver.b().onTrue(commands.autoScoreCommands.scoreSpeakerCommand);

    //scoringSubsystem.setDefaultCommand(commands.elevatorAutoHome);

    drivetrainSubsystem.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrainSubsystem
            .applyRequest(() -> drive.withVelocityX(-(driver.getLeftY() * Math.abs(driver.getLeftY())) * MaxSpeed) // Drive
                // forward
                // with
                // negative Y (forward)
                .withVelocityY(-(driver.getLeftX() * Math.abs(driver.getLeftX())) * MaxSpeed) // Drive left with
                                                                                              // negative X
                                                                                              // (left)
                .withRotationalRate(-(driver.getRightX() * Math.abs(driver.getRightX())) * MaxAngularRate) // Drive
                                                                                                           // counterclockwise
                                                                                                           // with
                                                                                                           // negative
                                                                                                           // X (left)
            ));

    driver.leftTrigger().whileTrue(drivetrainSubsystem.applyRequest(() -> brake));

    // reset the field-centric heading
    driver.start().onTrue(drivetrainSubsystem.runOnce(() -> drivetrainSubsystem.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrainSubsystem.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrainSubsystem.registerTelemetry(logger::telemeterize);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
