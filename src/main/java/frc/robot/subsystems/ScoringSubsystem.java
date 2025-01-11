package frc.robot.subsystems;//lol "package" HA AH AHAHAHAGGGG *Cough noise *Cough noise*5 *Falls down stairs... - Lukas

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ElevatorAutoHomeCommand;

public class ScoringSubsystem extends SubsystemBase {

    public double rightHomePos = Double.MAX_VALUE;
    public double leftHomePos = Double.MAX_VALUE;

    public TalonFX shooterTopMotor = new TalonFX(Constants.ShooterContants.SHOOTER_TOP_MOTOR_ID);

    StatusSignal<Double> shooterTopSpeed;
    StatusSignal<Double> shooterBotSpeed;

    public TalonFX shooterBottomMotor = new TalonFX(Constants.ShooterContants.SHOOTER_BOTTOM_MOTOR_ID);

    public TalonFX elevatorLeftMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_LEFT_MOTOR_ID);
    public TalonFX elevatorRightMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_RIGHT_MOTOR_ID);
    //DigitalInput toplimitSwitch;
    public DigitalInput bottomlimitSwitch;

    StatusSignal<Double> leftPos;
    StatusSignal<Double> rightPos;

    ElevatorAutoHomeCommand autoHomeCommand;

    public ScoringSubsystem() {
        //toplimitSwitch = new DigitalInput(8);
        bottomlimitSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_BOTTOM_SWITCH_ID);

        TalonFXConfiguration elevatorMotor1Config = new TalonFXConfiguration();
        elevatorMotor1Config.Slot0.kP = 0.7;
        elevatorMotor1Config.Slot0.kI = 0.5;
        elevatorMotor1Config.Slot0.kD = 0.1;
        elevatorLeftMotor.getConfigurator().apply(elevatorMotor1Config);
        elevatorLeftMotor.setInverted(false);
        leftPos = elevatorLeftMotor.getPosition();
        leftPos.setUpdateFrequency(50);
        elevatorLeftMotor.optimizeBusUtilization();
        //elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration elevatorMotor2Config = new TalonFXConfiguration();
        elevatorMotor2Config.Slot0.kP = 0.7;
        elevatorMotor2Config.Slot0.kI = 0.5;
        elevatorMotor2Config.Slot0.kD = 0.1;
        elevatorRightMotor.getConfigurator().apply(elevatorMotor2Config);
        elevatorRightMotor.setInverted(true);
        rightPos = elevatorRightMotor.getPosition();
        rightPos.setUpdateFrequency(50);
        elevatorRightMotor.optimizeBusUtilization();
        //elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration shooterTopConfig = new TalonFXConfiguration();
        shooterTopConfig.Slot0.kS = 0.1; // Add 0.05 V output to overcome static friction
        shooterTopConfig.Slot0.kV = 0.5; // A velocity target of 1 rps results in 0.12 V output
        shooterTopConfig.Slot0.kP = 1; // An error of 1 rps results in 0.11 V output
        shooterTopConfig.Slot0.kI = 0; // no output for integrated error
        shooterTopConfig.Slot0.kD = 0; // no output for error derivative
        shooterTopMotor.getConfigurator().apply(shooterTopConfig);
        shooterTopMotor.setInverted(false);
        shooterTopSpeed = shooterTopMotor.getVelocity();
        shooterTopSpeed.setUpdateFrequency(50);
        shooterTopMotor.optimizeBusUtilization();


        TalonFXConfiguration shooterBotConfig = new TalonFXConfiguration();
        shooterBotConfig.Slot0.kS = 0.1; // Add 0.05 V output to overcome static friction
        shooterBotConfig.Slot0.kV = 0.5; // A velocity target of 1 rps results in 0.12 V output
        shooterBotConfig.Slot0.kP = 1; // An error of 1 rps results in 0.11 V output
        shooterBotConfig.Slot0.kI = 0; // no output for integrated error
        shooterBotConfig.Slot0.kD = 0; // no output for error derivative
        shooterBottomMotor.getConfigurator().apply(shooterBotConfig);
        shooterBottomMotor.setInverted(false);
        shooterBotSpeed = shooterBottomMotor.getVelocity();
        shooterBotSpeed.setUpdateFrequency(50);
        shooterBottomMotor.optimizeBusUtilization();

        //autoHomeCommand = new ElevatorAutoHomeCommand(this);
    }

    @Override
    public void periodic() {
        if(leftHomePos == Double.MAX_VALUE || rightHomePos == Double.MAX_VALUE){
            
            this.elevatorLeftMotor.set(-0.1);
            this.elevatorRightMotor.set(-0.1);
            
            if (!this.bottomlimitSwitch.get()) {
                this.elevatorLeftMotor.set(0.0);
                this.elevatorRightMotor.set(0.0);
    
                StatusSignal<Double> leftPos = elevatorLeftMotor.getPosition();
                StatusSignal<Double> rightPos = elevatorRightMotor.getPosition();

                leftPos.waitForUpdate(1.0);
                rightPos.waitForUpdate(1.0);
        
                leftHomePos = leftPos.getValue();
                rightHomePos = rightPos.getValue();

                this.elevatorLeftMotor.setNeutralMode(NeutralModeValue.Brake);
                this.elevatorRightMotor.setNeutralMode(NeutralModeValue.Brake);
            }
        }

        Logger.recordOutput("Elevator At Bottom", !bottomlimitSwitch.get());

        Logger.recordOutput("Shooter Top Speed", shooterTopSpeed.refresh().getValue());
        Logger.recordOutput("Shooter Bot Speed", shooterBotSpeed.refresh().getValue());

        Logger.recordOutput("Elevator Left Pos", leftPos.refresh().getValue());
        Logger.recordOutput("Elevator Right Pos", rightPos.refresh().getValue());
    }

}
