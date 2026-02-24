package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.AlignToClimbPostCommand;
import frc.robot.commands.AlignToFuelHubCommand;
import frc.robot.commands.ShootFromDistanceCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BlinkinLedSubsystem;
import frc.robot.subsystems.ClimbVisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class RobotContainer {

    /* ================= CONTROLLER ================= */
    private final CommandXboxController controller =
            new CommandXboxController(0);

    /* ================= DRIVETRAIN ================= */
    public final CommandSwerveDrivetrain drivetrain =
            TunerConstants.createDrivetrain();
    private final ShooterFlywheelSubsystem shooterFlywheel =
            new ShooterFlywheelSubsystem();
    private final ConveyorSubsystem conveyor =
            new ConveyorSubsystem();
    private final ClimbVisionSubsystem climbVision =
            new ClimbVisionSubsystem();
    private final BlinkinLedSubsystem ledSubsystem =
            new BlinkinLedSubsystem();
    private final Telemetry telemetry =
            new Telemetry(MAX_SPEED);

    /* ================= CONSTANTS ================= */
    private static final double MAX_SPEED = 4.5;
    private static final double MAX_ROT   = 6.0;
    private static final double DEADBAND  = 0.10;
    private static final double TEST_SHOOT_RPM = 4200.0;
    private static final double SHOOT_CONVEYOR_PERCENT = 0.55;
    private static final double FUEL_HUB_ALIGN_TIMEOUT_SEC = 1.5;

    /* ================= OUTPOST POSITIONS ================= */
    private static final Pose2d BLUE_OUTPOST =
            new Pose2d(4.5, 3.0, new Rotation2d());

    private static final Pose2d RED_OUTPOST =
            new Pose2d(12.0, 3.0, Rotation2d.fromDegrees(180));

    /* ================= DRIVE REQUEST ================= */
    private final SwerveRequest.FieldCentric driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* ================= HEADING CONTROL ================= */
    private final PIDController headingPID =
            new PIDController(5.0, 0.0, 0.1);

    private boolean orbitMode = false;
    private boolean snapMode  = false;
    private double snapTarget = 0.0;

    public RobotContainer() {

        headingPID.enableContinuousInput(-Math.PI, Math.PI);

        configureBindings();
    }

    private void configureBindings() {

        /* ================= DEFAULT DRIVE ================= */
        drivetrain.setDefaultCommand(
                drivetrain.run(() -> {

                    double vx = applyDeadband(controller.getLeftY()) * MAX_SPEED;
                    double vy = applyDeadband(controller.getLeftX()) * MAX_SPEED;

                    double omega;

                    if (orbitMode) {
                        omega = calculateOrbitOmega();
                    }
                    else if (snapMode) {
                        omega = calculateSnapOmega();
                    }
                    else {
                        omega = applyDeadband(-controller.getRightX()) * MAX_ROT;
                    }

                    drivetrain.setControl(
                            driveRequest
                                    .withVelocityX(vx)
                                    .withVelocityY(vy)
                                    .withRotationalRate(omega)
                    );
                })
        );

        /* ================= ORBIT (Hold A) ================= */
        controller.a().whileTrue(
                Commands.startEnd(
                        () -> {
                            orbitMode = true;
                            snapMode = false;
                            headingPID.reset();
                        },
                        () -> orbitMode = false
                )
        );

        /* ================= SNAP (POV) ================= */
        controller.povUp().onTrue(Commands.runOnce(() -> setSnap(0)));
        controller.povRight().onTrue(Commands.runOnce(() -> setSnap(90)));
        controller.povDown().onTrue(Commands.runOnce(() -> setSnap(180)));
        controller.povLeft().onTrue(Commands.runOnce(() -> setSnap(270)));

        /* ================= MANUAL SQUARE TO HUB (Hold X) ================= */
        controller.x().whileTrue(
                new AlignToFuelHubCommand(drivetrain).repeatedly()
        );

        /* ================= CLIMB POST ALIGN (Hold Y) ================= */
        controller.y().whileTrue(
                new AlignToClimbPostCommand(drivetrain, climbVision, ledSubsystem).repeatedly()
        );

        /* ================= AUTO ALIGN + SHOOT (Hold Left Trigger) ================= */
        controller.leftTrigger(0.25).whileTrue(
                Commands.sequence(
                        new AlignToFuelHubCommand(drivetrain)
                                .withTimeout(FUEL_HUB_ALIGN_TIMEOUT_SEC),
                        new ShootFromDistanceCommand(
                                shooterFlywheel,
                                conveyor,
                                this::getDistanceToFuelHubMeters,
                                SHOOT_CONVEYOR_PERCENT
                        )
                )
        );

        /* ================= SHOOT TEST (Right Trigger) ================= */
        controller.rightTrigger(0.25).whileTrue(
                Commands.startEnd(
                        () -> shooterFlywheel.setTargetRPM(TEST_SHOOT_RPM),
                        shooterFlywheel::stop,
                        shooterFlywheel
                )
        );
    }

    /* ================= ORBIT LOGIC ================= */

    private double calculateOrbitOmega() {

        Pose2d cur = drivetrain.getPose();
        boolean isBlue = isBlueAlliance();

        double hubX = isBlue ? 4.5 : 12.0;
        double hubY = 3.0;

        double dx = hubX - cur.getX();
        double dy = hubY - cur.getY();

        double targetAngle = Math.atan2(dy, dx);

        return MathUtil.clamp(
                headingPID.calculate(
                        cur.getRotation().getRadians(),
                        targetAngle
                ),
                -MAX_ROT, MAX_ROT
        );
    }

    /* ================= SNAP LOGIC ================= */

    private void setSnap(double degrees) {
        snapTarget = Math.toRadians(degrees);
        snapMode = true;
        orbitMode = false;
        headingPID.reset();
    }

    private double calculateSnapOmega() {

        double current = drivetrain.getPose().getRotation().getRadians();
        double error = MathUtil.angleModulus(snapTarget - current);

        double omega = headingPID.calculate(current, snapTarget);

        if (Math.abs(error) < Math.toRadians(2.0)) {
            snapMode = false;
            return 0.0;
        }

        return MathUtil.clamp(omega, -MAX_ROT, MAX_ROT);
    }

    /* ================= SUPPLIERS ================= */

    private DoubleSupplier driverX() {
        return () -> applyDeadband(-controller.getLeftY()) * MAX_SPEED;
    }

    private DoubleSupplier driverY() {
        return () -> applyDeadband(-controller.getLeftX()) * MAX_SPEED;
    }

    /* ================= HELPERS ================= */

    private Pose2d getAllianceOutpostPose() {
        return isBlueAlliance() ? BLUE_OUTPOST : RED_OUTPOST;
    }

    private boolean isBlueAlliance() {
        return DriverStation.getAlliance()
                .orElse(Alliance.Blue) == Alliance.Blue;
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }

    private double getDistanceToFuelHubMeters() {
        Translation2d hubCenter = FieldGeometry.getFuelHubCenter();
        return drivetrain.getPose().getTranslation().getDistance(hubCenter);
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public void updateTelemetry() {
        telemetry.telemeterize(drivetrain.getState());
    }

    public void setLedState(BlinkinLedSubsystem.LedState state) {
        ledSubsystem.setState(state);
    }
}
