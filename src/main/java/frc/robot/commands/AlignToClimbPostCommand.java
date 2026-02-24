package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BlinkinLedSubsystem;
import frc.robot.subsystems.BlinkinLedSubsystem.LedState;
import frc.robot.subsystems.ClimbVisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToClimbPostCommand extends Command {

    private static final double TX_TOLERANCE_DEG = 1.0;
    private static final int REQUIRED_STABLE_CYCLES = 5;
    private static final double MAX_ROT_RAD_PER_SEC = 2.0;
    private static final double SEARCH_ROT_RAD_PER_SEC = 0.6;

    private final CommandSwerveDrivetrain drivetrain;
    private final ClimbVisionSubsystem climbVision;
    private final BlinkinLedSubsystem leds;
    private final PIDController txPid = new PIDController(0.06, 0.0, 0.001);
    private final SwerveRequest.FieldCentric rotateRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private int stableCycles = 0;

    public AlignToClimbPostCommand(
        CommandSwerveDrivetrain drivetrain,
        ClimbVisionSubsystem climbVision,
        BlinkinLedSubsystem leds
    ) {
        this.drivetrain = drivetrain;
        this.climbVision = climbVision;
        this.leds = leds;
        addRequirements(drivetrain, climbVision);
    }

    @Override
    public void initialize() {
        stableCycles = 0;
        txPid.reset();

        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        int selectedPipeline = isBlue
            ? Constants.Vision.Climb.BLUE_PIPELINE_INDEX
            : Constants.Vision.Climb.RED_PIPELINE_INDEX;
        climbVision.setPipelineIndex(selectedPipeline);

        SmartDashboard.putString("ClimbAlign/State", "Aligning");
        SmartDashboard.putNumber("ClimbAlign/SelectedPipeline", selectedPipeline);
        leds.setState(LedState.CLIMB_ALIGNING);
    }

    @Override
    public void execute() {
        if (!climbVision.hasTarget()) {
            stableCycles = 0;
            drivetrain.setControl(
                rotateRequest
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(SEARCH_ROT_RAD_PER_SEC)
            );
            SmartDashboard.putString("ClimbAlign/State", "Searching");
            SmartDashboard.putNumber("ClimbAlign/ErrorDeg", 0.0);
            leds.setState(LedState.CLIMB_SEARCHING);
            return;
        }

        double txErrorDeg = climbVision.getTxDegrees();
        double omega = MathUtil.clamp(
            txPid.calculate(txErrorDeg, 0.0),
            -MAX_ROT_RAD_PER_SEC,
            MAX_ROT_RAD_PER_SEC
        );

        drivetrain.setControl(
            rotateRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(omega)
        );

        SmartDashboard.putString("ClimbAlign/State", "Aligning");
        SmartDashboard.putNumber("ClimbAlign/ErrorDeg", txErrorDeg);

        if (Math.abs(txErrorDeg) <= TX_TOLERANCE_DEG) {
            stableCycles++;
            leds.setState(LedState.CLIMB_ALIGNED);
        } else {
            stableCycles = 0;
            leds.setState(LedState.CLIMB_ALIGNING);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        SmartDashboard.putString("ClimbAlign/State", "Idle");
        restoreModeLedState();
    }

    @Override
    public boolean isFinished() {
        return stableCycles >= REQUIRED_STABLE_CYCLES;
    }

    private void restoreModeLedState() {
        if (DriverStation.isDisabled()) {
            leds.setState(LedState.DISABLED);
        } else if (DriverStation.isAutonomousEnabled()) {
            leds.setState(LedState.AUTONOMOUS);
        } else if (DriverStation.isTestEnabled()) {
            leds.setState(LedState.TEST);
        } else {
            leds.setState(LedState.TELEOP);
        }
    }
}
