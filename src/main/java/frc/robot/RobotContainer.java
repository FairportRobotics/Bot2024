// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberOffCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.ElevatorDownCommand;
import frc.robot.commands.ElevatorOffCommand;
import frc.robot.commands.ElevatorUpCommand;
import frc.robot.commands.FeederOffCommand;
import frc.robot.commands.FeederOnCommand;
import frc.robot.commands.IntakeNoteToFeederCommand;
import frc.robot.commands.IntakeOffCommand;
import frc.robot.commands.IntakeOnCommand;
import frc.robot.commands.ShooterOffCommand;
import frc.robot.commands.ShooterOnCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ScoringSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
      Constants.OperatorConstants.kOperatiorControllerPort); // operator SHOULD BE ON 1 BUT CONFLICTS
  // IN SIM
  private final CommandXboxController driver = new CommandXboxController(
      Constants.OperatorConstants.kDriverControllerPort); // driver
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final ScoringSubsystem Score = new ScoringSubsystem();
  private final IntakeSubsystem Intake = new IntakeSubsystem();
  private final ClimberSubsystem climb = new ClimberSubsystem();

  private final SendableChooser<Boolean> bindingChooser = new SendableChooser<Boolean>();

  public class Commands {
    public Command intakeOnCommand = new IntakeOnCommand(Intake, 0.3);
    public Command intakeOffCommand = new IntakeOffCommand(Intake);

    public Command feederFwdCommand = new FeederOnCommand(Intake, 0.25);
    public Command feederOffCommand = new FeederOffCommand(Intake);
    public Command feederRevCommand = new FeederOnCommand(Intake, -0.25);

    public Command shooterOnCommand = new ShooterOnCommand(Score, 0.75);
    public Command shooterOffCommand = new ShooterOffCommand(Score);

    public Command elevatorUpCommand = new ElevatorUpCommand(Score, 0.1);
    public Command elevatorDownCommand = new ElevatorDownCommand(Score, 0.1);
    public Command elevatorOffCommand = new ElevatorOffCommand(Score);

    public Command climberUpCommand = new ClimberUpCommand(climb, 0.75);
    public Command climberDownCommand = new ClimberDownCommand(climb, 0.75);
    public Command climberOffCommand = new ClimberOffCommand(climb);
  }

  Commands commands = new Commands();

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

    bindingChooser.addOption("System Check", true);
    bindingChooser.setDefaultOption("Match", false);

    SmartDashboard.putData(bindingChooser);

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

    //if (bindingChooser.getSelected() && !DriverStation.isFMSAttached()) {
      if (true) {
      // SYSTEM CHECK BINDINGS

      // here for Scoring subsystem :) change if too low or high
      operator.x().onTrue(commands.shooterOnCommand);
      operator.x().onFalse(commands.shooterOffCommand);

      operator.rightBumper().onTrue(commands.intakeOnCommand);
      operator.rightBumper().onFalse(commands.intakeOffCommand);
      // here for intake subsystem :)
      operator.leftBumper().onTrue(commands.feederFwdCommand);
      operator.leftBumper().onFalse(commands.feederOffCommand);
      operator.y().onTrue(commands.feederRevCommand);
      // Climber
      operator.rightTrigger().onTrue(commands.climberUpCommand);
      operator.rightTrigger().onFalse(commands.climberOffCommand);
      operator.leftTrigger().onTrue(commands.climberDownCommand);
      operator.leftTrigger().onFalse(commands.climberOffCommand);

      operator.a().onTrue(commands.elevatorUpCommand);
      operator.a().onFalse(commands.elevatorOffCommand);
      operator.b().onTrue(commands.elevatorDownCommand);
      operator.b().onFalse(commands.elevatorOffCommand);

      // //path

      // reset the field-centric heading on left bumper press
      // operator.x().onTrue(drivetrain.runOnce(() ->
      // drivetrain.seedFieldRelative())); //COMMENTED OUT DUE TO BUTTON CONFLICTS
    } else {
      // NORMAL DRIVE MODE BINDINGS

      operator.a().onTrue(new IntakeNoteToFeederCommand(Intake));

      driver.povDown().onTrue(CommandSwerveDrivetrain.fastestAutoCommand()); // "Fastest" auto
      driver.povUp().onTrue(CommandSwerveDrivetrain.autoAutoCommand()); // "Auto" auto
      // driver.povRight().onTrue(CommandSwerveDrivetrain.autoAutoCommand()); //
      // "Auto" autog
      // driver.povLeft().onTrue(CommandSwerveDrivetrain.fastestAutoCommand()); //
      // "Fastest" auto
    }

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-(driver.getLeftY() * Math.abs(driver.getLeftY())) * MaxSpeed) // Drive
                                                                                                                         // forward
                                                                                                                         // with
            // negative Y (forward)
            .withVelocityY(-(driver.getLeftX() * Math.abs(driver.getLeftX())) * MaxSpeed) // Drive left with negative X
                                                                                          // (left)
            .withRotationalRate(-(driver.getRightX() * Math.abs(driver.getRightX())) * MaxAngularRate) // Drive
                                                                                                       // counterclockwise
                                                                                                       // with negative
                                                                                                       // X (left)
        ));

    driver.a().whileTrue(drivetrain.applyRequest(() -> brake));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return CommandSwerveDrivetrain.autoAutoCommand();
  }
}
