package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollerSubsystem extends SubsystemBase {

    private final SparkFlex intakeMotor;

    @SuppressWarnings("removal")
    public IntakeRollerSubsystem() {
        intakeMotor = new SparkFlex(Constants.CanIds.INTAKE_ROLLER, MotorType.kBrushless); // rio CAN

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(50);

        intakeMotor.configure(
            config,
            SparkFlex.ResetMode.kResetSafeParameters,
            SparkFlex.PersistMode.kPersistParameters
        );
    }

    // + forward, - reverse
    public void run(double percent) {
        intakeMotor.set(percent);
    }

    public void stop() {
        intakeMotor.set(0);
    }
}
