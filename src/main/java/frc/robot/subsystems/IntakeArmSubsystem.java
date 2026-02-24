package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArmSubsystem extends SubsystemBase {

    private final SparkFlex armMotor;
    private final SparkClosedLoopController armPID;

    private double targetPosition = 0.0;

    @SuppressWarnings("removal")
    public IntakeArmSubsystem() {

        armMotor = new SparkFlex(Constants.CanIds.INTAKE_ARM, MotorType.kBrushless); // rio CAN

        SparkFlexConfig config = new SparkFlexConfig();

        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(60);

        // TODO: Tune these
        config.closedLoop.p(0.3);
        config.closedLoop.i(0);
        config.closedLoop.d(0);

        armMotor.configure(
            config,
            SparkFlex.ResetMode.kResetSafeParameters,
            SparkFlex.PersistMode.kPersistParameters
        );

        armPID = armMotor.getClosedLoopController();

        // Hold current position at startup
        targetPosition = armMotor.getEncoder().getPosition();
    }

    // Move to a position (rotations)
    @SuppressWarnings("removal")
    public void setPosition(double position) {
        targetPosition = position;
        armPID.setReference(position, SparkFlex.ControlType.kPosition);
    }

    // Manual control (driver override)
    public void manualMove(double percent) {
        armMotor.set(percent);
        targetPosition = armMotor.getEncoder().getPosition();
    }

    public double getPosition() {
        return armMotor.getEncoder().getPosition();
    }

    @SuppressWarnings("removal")
    @Override
    public void periodic() {
        // Always hold last commanded position
        armPID.setReference(targetPosition, SparkFlex.ControlType.kPosition);
    }
}
