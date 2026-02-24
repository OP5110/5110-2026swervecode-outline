package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterLoaderSubsystem extends SubsystemBase {

    private final SparkFlex loaderMotor;

    @SuppressWarnings("removal")
    public ShooterLoaderSubsystem() {

        loaderMotor = new SparkFlex(Constants.CanIds.SHOOTER_LOADER, MotorType.kBrushless); // rio CAN

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(50);

        loaderMotor.configure(
            config,
            SparkFlex.ResetMode.kResetSafeParameters,
            SparkFlex.PersistMode.kPersistParameters
        );
    }

    // Forward only
    public void run(double percent) {
        loaderMotor.set(Math.max(0, percent));  // Prevent reverse
    }

    public void stop() {
        loaderMotor.set(0);
    }
}
