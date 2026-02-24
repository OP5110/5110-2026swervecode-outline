package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterFlywheelSubsystem;

public class ShootFromDistanceCommand extends Command {

    private final ShooterFlywheelSubsystem shooter;
    private final ConveyorSubsystem conveyor;
    private final DoubleSupplier distanceMetersSupplier;
    private final double conveyorPercent;

    public ShootFromDistanceCommand(
        ShooterFlywheelSubsystem shooter,
        ConveyorSubsystem conveyor,
        DoubleSupplier distanceMetersSupplier,
        double conveyorPercent
    ) {
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.distanceMetersSupplier = distanceMetersSupplier;
        this.conveyorPercent = MathUtil.clamp(conveyorPercent, 0.0, 1.0);

        addRequirements(shooter, conveyor);
    }

    @Override
    public void initialize() {
        conveyor.stop();
        SmartDashboard.putString("AutoShoot/State", "SpinningUp");
    }

    @Override
    public void execute() {
        double distanceMeters = distanceMetersSupplier.getAsDouble();
        shooter.setTargetFromDistanceMeters(distanceMeters);
        SmartDashboard.putNumber("AutoShoot/HubDistanceMeters", distanceMeters);

        if (shooter.atSpeed()) {
            conveyor.run(conveyorPercent);
            SmartDashboard.putString("AutoShoot/State", "Feeding");
        } else {
            conveyor.stop();
            SmartDashboard.putString("AutoShoot/State", "SpinningUp");
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.stop();
        shooter.stop();
        SmartDashboard.putString("AutoShoot/State", "Idle");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
