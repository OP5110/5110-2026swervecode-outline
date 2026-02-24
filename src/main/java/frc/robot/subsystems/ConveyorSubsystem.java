package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {

    private final SparkFlex conveyorMotor;

    @SuppressWarnings("removal")
    public ConveyorSubsystem() {

        conveyorMotor = new SparkFlex(Constants.CanIds.CONVEYOR, MotorType.kBrushless); // rio CAN ID

        SparkFlexConfig config = new SparkFlexConfig();

        config.idleMode(IdleMode.kBrake);     // Brake helps prevent rollback
        config.smartCurrentLimit(50);        // Safe for Vortex

        conveyorMotor.configure(
            config,
            SparkFlex.ResetMode.kResetSafeParameters,
            SparkFlex.PersistMode.kPersistParameters
        );
    }

    // + forward, - reverse
    public void run(double percent) {
        conveyorMotor.set(percent);
    }

    public void stop() {
        conveyorMotor.set(0);
    }
}
