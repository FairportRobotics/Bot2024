package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // Since we are using a holonomic drivetrain, the rotation component of this
    // pose
    // represents the goal holonomic rotation
    public Pose2d roboSpeakerPose2d = new Pose2d(2.11, 7.17, Rotation2d.fromDegrees(180.0));// Speaker pos according to pathplanner
    public Pose2d roboAmpPose2d = new Pose2d(1.78, 5.41, Rotation2d.fromDegrees(-90.0));// amp pos according to pathplanner

    // Create the constraints to use while pathfinding
    public PathConstraints constraints = new PathConstraints(
            3.0, 2.0,
            Units.degreesToRadians(270), Units.degreesToRadians(360));

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void periodic() {
        Subsystem.super.periodic();

        LimelightResults results = LimelightHelpers.getLatestResults("MasonIsAGoober");
        Pose2d visionPose = results.targetingResults.getBotPose2d_wpiBlue();
        if (visionPose.getX() == 0.0){
            return;
        }

        double latency = Timer.getFPGATimestamp() - (results.targetingResults.botpose_wpiblue[6]/1000.0);
        this.addVisionMeasurement(visionPose, latency);
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

    private SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();

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

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

}
