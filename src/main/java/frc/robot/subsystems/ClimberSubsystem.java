package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX climberMotor;
    private final Servo climberLatchServo;

    // Control Requests
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0).withEnableFOC(true);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);

    private double holdPosition = 0.0;
    private boolean holding = false;
    private double latchReleaseReadyTimeSec = 0.0;

    public ClimberSubsystem() {

        climberMotor = new TalonFX(Constants.CanIds.CLIMBER_MOTOR, Constants.CanBus.RIO); // Use rio bus unless you intentionally move it
        climberLatchServo = new Servo(Constants.PwmPorts.CLIMBER_LATCH_SERVO);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // Brake mode is CRITICAL
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Current limiting (tune as needed)
        config.CurrentLimits.SupplyCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // Position PID (STARTING VALUES – MUST TUNE)
        config.Slot0.kP = 40;    // Kraken position values are large
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        climberMotor.getConfigurator().apply(config);

        // Capture initial hold position
        holdPosition = climberMotor.getPosition().getValueAsDouble();

        // Default to locked so the gear cannot spin freely at boot.
        engageLatch();
    }

    // ---------------- Manual Up/Down ----------------

    public void run(double percent) {
        if (Math.abs(percent) < 0.01) {
            stopAndHold();
            return;
        }

        if (isLatchEngaged()) {
            disengageLatch();
            latchReleaseReadyTimeSec =
                Timer.getFPGATimestamp() + Constants.Climber.LATCH_RELEASE_DELAY_SEC;
        }

        if (Timer.getFPGATimestamp() < latchReleaseReadyTimeSec) {
            climberMotor.setControl(dutyRequest.withOutput(0.0));
            return;
        }

        holding = false;
        climberMotor.setControl(dutyRequest.withOutput(percent));
    }

    public void stopAndHold() {
        holding = false;
        engageLatch();
        climberMotor.setControl(dutyRequest.withOutput(0.0));
    }

    public void engageLatch() {
        climberLatchServo.set(Constants.Climber.LATCH_ENGAGED_POSITION);
    }

    public void disengageLatch() {
        climberLatchServo.set(Constants.Climber.LATCH_DISENGAGED_POSITION);
    }

    public void toggleLatch() {
        if (isLatchEngaged()) {
            disengageLatch();
        } else {
            engageLatch();
        }
    }

    public boolean isLatchEngaged() {
        double current = climberLatchServo.get();
        double target = Constants.Climber.LATCH_ENGAGED_POSITION;
        return Math.abs(current - target) < 0.02;
    }

    // ---------------- Periodic Hold ----------------

    @Override
    public void periodic() {

        if (holding) {
            climberMotor.setControl(
                positionRequest.withPosition(holdPosition)
            );
        }
    }

    // ---------------- Utility ----------------

    public double getPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }
}
