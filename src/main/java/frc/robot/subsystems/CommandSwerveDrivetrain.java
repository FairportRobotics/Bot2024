package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LimelightHelpers.LimelightResults;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends LegacySwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = llTable.getEntry("tx");
    private NetworkTableEntry ty = llTable.getEntry("ty");
    private NetworkTableEntry ta = llTable.getEntry("ta");

    private NetworkTableEntry tl = llTable.getEntry("tl");
    private NetworkTableEntry cl = llTable.getEntry("tl");

    private NetworkTableEntry botPose = llTable.getEntry("botpose_wpiblue");

    private NetworkTableEntry tid = llTable.getEntry("tid");

    // Since we are using a holonomic drivetrain, the rotation component of this
    // pose
    // represents the goal holonomic rotation
    public Pose2d roboSpeakerPose2d = new Pose2d(2.11, 7.17, Rotation2d.fromDegrees(180.0));// Speaker pos according to
                                                                                            // pathplanner
    public Pose2d roboAmpPose2d = new Pose2d(1.78, 5.41, Rotation2d.fromDegrees(-90.0));// amp pos according to
                                                                                        // pathplanner

    // Create the constraints to use while pathfinding
    public PathConstraints constraints = new PathConstraints(
            3.0, 2.0,
            Units.degreesToRadians(270), Units.degreesToRadians(360));

    public CommandSwerveDrivetrain(LegacySwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            LegacySwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;

        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given
                                                                          // ROBOT RELATIVE ChassisSpeeds. Also
                                                                          // optionally outputs individual module
                                                                          // feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
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
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(LegacySwerveDrivetrainConstants driveTrainConstants,
            LegacySwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given
                                                                          // ROBOT RELATIVE ChassisSpeeds. Also
                                                                          // optionally outputs individual module
                                                                          // feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
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
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void periodic() {
        Subsystem.super.periodic();

        Logger.recordOutput("Odemetry Pose", getPose());

        Logger.recordOutput("SwerveModuleState", this.m_moduleStates);

        double[] botPoseVals = botPose.getDoubleArray(new double[5]);

        if (botPoseVals[0] == 0.0) {
            return;
        }

        // double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // // how many degrees back is your limelight rotated from perfectly vertical?
        // double limelightMountAngleDegrees = 15.0;

        // // distance from the center of the Limelight lens to the floor
        // double limelightLensHeightInches = 24.8;

        // // distance from the target to the floor
        // double goalHeightInches = 0;

        // long currentID = tid.getInteger(-1);

        // if(currentID == 7 || currentID == 8 || currentID == 3 || currentID ==
        // 4)//Speaker tags
        // {
        // goalHeightInches = 53.88;
        // }
        // else if(currentID == 5 || currentID == 6)//Amp tags
        // {
        // goalHeightInches = 50.13;
        // }
        // else if(currentID == 11 || currentID == 12 || currentID == 13 || currentID ==
        // 14 || currentID == 15 || currentID == 16)//Stage tags
        // {
        // goalHeightInches = 48.81;
        // }

        // double angleToGoalDegrees = limelightMountAngleDegrees +
        // targetOffsetAngle_Vertical;
        // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // // calculate distance
        // double distanceFromLimelightToGoalInches = (goalHeightInches -
        // limelightLensHeightInches)
        // / Math.tan(angleToGoalRadians);

        Pose2d pose = new Pose2d(botPoseVals[0], botPoseVals[1], Rotation2d.fromDegrees(botPoseVals[5]));

        if (DriverStation.isAutonomous()) {
            double latency = Timer.getFPGATimestamp() - (botPoseVals[6] / 1000.0);
            this.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
            this.addVisionMeasurement(pose, latency);
        }

        Logger.recordOutput("Limelight Pose", pose);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private LegacySwerveRequest.ApplyChassisSpeeds drive = new LegacySwerveRequest.ApplyChassisSpeeds();

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public void resetPose(Pose2d newPose) {
        this.seedFieldRelative(newPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.m_kinematics.toChassisSpeeds(this.getState().ModuleStates);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.setControl(drive.withSpeeds(speeds));
    }

    public Command applyRequest(Supplier<LegacySwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

}
