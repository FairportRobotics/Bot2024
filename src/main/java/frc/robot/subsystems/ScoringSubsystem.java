package frc.robot.subsystems;//lol "package" HA AH AHAHAHAGGGG *Cough noise *Cough noise*5 *Falls down stairs... - Lukas

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ScoringSubsystem extends SubsystemBase {

    public double rightHomePos = Double.MAX_VALUE;
    public double leftHomePos = Double.MAX_VALUE;

    public TalonFX shooterTopMotor = new TalonFX(Constants.ShooterContants.SHOOTER_TOP_MOTOR_ID);
    public TalonFX shooterBottomMotor = new TalonFX(Constants.ShooterContants.SHOOTER_BOTTOM_MOTOR_ID);

    public TalonFX elevatorLeftMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_LEFT_MOTOR_ID);
    public TalonFX elevatorRightMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_RIGHT_MOTOR_ID);
    //DigitalInput toplimitSwitch;
    public DigitalInput bottomlimitSwitch;

    public ScoringSubsystem() {
        //toplimitSwitch = new DigitalInput(8);
        bottomlimitSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_BOTTOM_SWITCH_ID);

        TalonFXConfiguration elevatorMotor1Config = new TalonFXConfiguration();
        elevatorMotor1Config.Slot0.kP = 24;
        elevatorMotor1Config.Slot0.kI = 0;
        elevatorMotor1Config.Slot0.kD = 0.1;
        elevatorLeftMotor.getConfigurator().apply(elevatorMotor1Config);
        elevatorLeftMotor.setInverted(false);

        TalonFXConfiguration elevatorMotor2Config = new TalonFXConfiguration();
        elevatorMotor2Config.Slot0.kP = 24;
        elevatorMotor2Config.Slot0.kI = 0;
        elevatorMotor2Config.Slot0.kD = 0.1;
        elevatorRightMotor.getConfigurator().apply(elevatorMotor2Config);
        elevatorRightMotor.setInverted(true);

        TalonFXConfiguration shooterTopConfig = new TalonFXConfiguration();
        shooterTopConfig.Slot0.kS = 0.05; // Add 0.05 V output to overcome static friction
        shooterTopConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        shooterTopConfig.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
        shooterTopConfig.Slot0.kI = 0; // no output for integrated error
        shooterTopConfig.Slot0.kD = 0; // no output for error derivative
        shooterTopMotor.getConfigurator().apply(shooterTopConfig);
        shooterTopMotor.setInverted(false);


        TalonFXConfiguration shooterBotConfig = new TalonFXConfiguration();
        shooterBotConfig.Slot0.kS = 0.05; // Add 0.05 V output to overcome static friction
        shooterBotConfig.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        shooterBotConfig.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
        shooterBotConfig.Slot0.kI = 0; // no output for integrated error
        shooterBotConfig.Slot0.kD = 0; // no output for error derivative
        shooterBottomMotor.getConfigurator().apply(shooterTopConfig);
        shooterBottomMotor.setInverted(false);

    }

}
