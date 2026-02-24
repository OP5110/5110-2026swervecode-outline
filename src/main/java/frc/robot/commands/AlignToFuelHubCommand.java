package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.FieldGeometry;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToFuelHubCommand extends Command {

    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(2.0);
    private static final int REQUIRED_STABLE_CYCLES = 5;
    private static final double MAX_ROT_RAD_PER_SEC = 5.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController headingPid = new PIDController(5.0, 0.0, 0.1);

    private final SwerveRequest.FieldCentric holdAndTurnRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private int stableCycles = 0;

    public AlignToFuelHubCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        stableCycles = 0;

        headingPid.enableContinuousInput(-Math.PI, Math.PI);
        headingPid.reset();

        SmartDashboard.putString("AutoShoot/State", "Aligning");
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();
        Translation2d hubCenter = FieldGeometry.getFuelHubCenter();

        double targetHeading = Math.atan2(
            hubCenter.getY() - robotPose.getY(),
            hubCenter.getX() - robotPose.getX()
        );

        double currentHeading = robotPose.getRotation().getRadians();
        double headingError = MathUtil.angleModulus(targetHeading - currentHeading);
        double distanceMeters = robotPose.getTranslation().getDistance(hubCenter);

        double omega = MathUtil.clamp(
            headingPid.calculate(currentHeading, targetHeading),
            -MAX_ROT_RAD_PER_SEC,
            MAX_ROT_RAD_PER_SEC
        );

        drivetrain.setControl(
            holdAndTurnRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(omega)
        );

        SmartDashboard.putNumber("AutoShoot/HubDistanceMeters", distanceMeters);
        SmartDashboard.putNumber("AutoShoot/AlignErrorDeg", Math.toDegrees(headingError));

        if (Math.abs(headingError) <= HEADING_TOLERANCE_RAD) {
            stableCycles++;
        } else {
            stableCycles = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        SmartDashboard.putString("AutoShoot/State", "Idle");
    }

    @Override
    public boolean isFinished() {
        return stableCycles >= REQUIRED_STABLE_CYCLES;
    }
}
