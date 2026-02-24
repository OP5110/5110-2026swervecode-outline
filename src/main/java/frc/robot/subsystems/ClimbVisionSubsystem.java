package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class ClimbVisionSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = Constants.Vision.Climb.LIMELIGHT_NAME;

    private final NetworkTable limelightTable =
        NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);

    private final NetworkTableEntry tvEntry = limelightTable.getEntry("tv");
    private final NetworkTableEntry txEntry = limelightTable.getEntry("tx");
    private final NetworkTableEntry taEntry = limelightTable.getEntry("ta");
    private final NetworkTableEntry getPipeEntry = limelightTable.getEntry("getpipe");

    public boolean hasTarget() {
        return tvEntry.getDouble(0.0) == 1.0;
    }

    public double getTxDegrees() {
        return txEntry.getDouble(0.0);
    }

    public double getTaPercent() {
        return taEntry.getDouble(0.0);
    }

    public void setPipelineIndex(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, pipelineIndex);
    }

    public int getActivePipelineIndex() {
        return (int) getPipeEntry.getDouble(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ClimbVision/HasTarget", hasTarget());
        SmartDashboard.putNumber("ClimbVision/TxDeg", getTxDegrees());
        SmartDashboard.putNumber("ClimbVision/Ta", getTaPercent());
        SmartDashboard.putNumber("ClimbVision/Pipeline", getActivePipelineIndex());
    }
}
