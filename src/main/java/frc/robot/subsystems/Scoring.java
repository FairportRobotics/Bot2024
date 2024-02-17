package frc.robot.subsystems;//lol "package" HA AH AHAHAHAGGGG *Cough noise *Cough noise*5 *Falls down stairs... - Lukas

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Scoring extends SubsystemBase {

    TalonFX Shoot = new TalonFX(33);
    TalonFX elevator = new TalonFX(17);
    DigitalInput toplimitSwitch;
    DigitalInput bottomlimitSwitch;

    public Scoring() {
        toplimitSwitch = new DigitalInput(8);
        bottomlimitSwitch = new DigitalInput(9);
    }

    public Command ScoringMethodCommand() {
        // Inline construction of command goes here.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    public boolean ScoringCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    // SHOOTER
    public Command Shoot(double shootforce) {
        return new Command() {
            @Override
            public void execute() {
                Shoot.set(shootforce);
            }
        };
    }

    // Elevator
    public Command elevatorUp() {
        return new Command() {
            @Override
            public void execute() {
                elevator.set(1);
            }

            @Override
            public void end(boolean interrupted) {
                elevator.set(0);
            }
        };
    }

    public Command elevatorDown() {
        return new Command() {
            @Override
            public void execute() {
                elevator.set(-1);
            }

            @Override
            public void end(boolean interrupted) {
                elevator.set(0);
            }
        };
    }

    // Elevator limit switches
    public void setMotorSpeed(double speed) {
        if (speed > 0) {
            if (toplimitSwitch.get()) {
                elevator.set(0);
            } else {
                elevator.set(speed);
            }
        } else {
            if (bottomlimitSwitch.get()) {
                elevator.set(0);
            } else {
                elevator.set(speed);
            }
        }
    }
}