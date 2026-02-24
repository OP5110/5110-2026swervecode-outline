package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterFlywheelSubsystem extends SubsystemBase {

    // Safe operating bounds for a single Vortex shooter
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6200.0;
    private static final double DEFAULT_AT_SPEED_TOLERANCE_RPM = 100.0;

    // Distance (m) to RPM table. Tune on-robot.
    private static final double[] DISTANCE_BREAKPOINTS_METERS = {
        1.0, 1.5, 2.0, 2.5, 3.0, 3.5
    };

    private static final double[] RPM_BREAKPOINTS = {
        2600, 3100, 3600, 4200, 4800, 5400
    };

    private final SparkFlex flywheelMotor;
    private final SparkClosedLoopController flywheelVelocityController;

    private double targetRPM = 0.0;
    private boolean velocityEnabled = false;

    @SuppressWarnings("removal")
    public ShooterFlywheelSubsystem() {
        flywheelMotor = new SparkFlex(Constants.CanIds.SHOOTER_FLYWHEEL, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(80);

        // Start-point tuning values for load disturbance rejection.
        config.closedLoop.p(0.00045);
        config.closedLoop.i(0.0);
        config.closedLoop.d(0.0);
        config.closedLoop.velocityFF(0.00020);

        flywheelMotor.configure(
            config,
            SparkFlex.ResetMode.kResetSafeParameters,
            SparkFlex.PersistMode.kPersistParameters
        );

        flywheelVelocityController = flywheelMotor.getClosedLoopController();
    }

    @SuppressWarnings("removal")
    public void setTargetRPM(double rpm) {
        targetRPM = MathUtil.clamp(rpm, MIN_RPM, MAX_RPM);

        if (targetRPM <= 0.0) {
            stop();
            return;
        }

        velocityEnabled = true;
        flywheelVelocityController.setReference(
            targetRPM,
            SparkFlex.ControlType.kVelocity
        );
    }

    public void setTargetFromDistanceMeters(double distanceMeters) {
        setTargetRPM(getRecommendedRPMForDistanceMeters(distanceMeters));
    }

    public double getRecommendedRPMForDistanceMeters(double distanceMeters) {
        if (DISTANCE_BREAKPOINTS_METERS.length == 0 || RPM_BREAKPOINTS.length == 0) {
            return 0.0;
        }

        if (distanceMeters <= DISTANCE_BREAKPOINTS_METERS[0]) {
            return RPM_BREAKPOINTS[0];
        }

        int last = DISTANCE_BREAKPOINTS_METERS.length - 1;
        if (distanceMeters >= DISTANCE_BREAKPOINTS_METERS[last]) {
            return RPM_BREAKPOINTS[last];
        }

        for (int i = 1; i < DISTANCE_BREAKPOINTS_METERS.length; i++) {
            if (distanceMeters <= DISTANCE_BREAKPOINTS_METERS[i]) {
                double x0 = DISTANCE_BREAKPOINTS_METERS[i - 1];
                double x1 = DISTANCE_BREAKPOINTS_METERS[i];
                double y0 = RPM_BREAKPOINTS[i - 1];
                double y1 = RPM_BREAKPOINTS[i];

                double t = (distanceMeters - x0) / (x1 - x0);
                return y0 + t * (y1 - y0);
            }
        }

        return RPM_BREAKPOINTS[last];
    }

    public double getCurrentRPM() {
        return flywheelMotor.getEncoder().getVelocity();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean atSpeed() {
        return atSpeed(DEFAULT_AT_SPEED_TOLERANCE_RPM);
    }

    public boolean atSpeed(double toleranceRPM) {
        return velocityEnabled
            && Math.abs(getCurrentRPM() - targetRPM) <= toleranceRPM;
    }

    public void stop() {
        velocityEnabled = false;
        targetRPM = 0.0;
        flywheelMotor.set(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/CurrentRPM", getCurrentRPM());
        SmartDashboard.putBoolean("Shooter/AtSpeed", atSpeed());
    }
}
