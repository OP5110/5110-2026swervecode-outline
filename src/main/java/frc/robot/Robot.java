package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.BlinkinLedSubsystem.LedState;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer = new RobotContainer();

    @Override
    public void robotInit() {
        // Nothing extra here; RobotContainer already schedules vision updates
        m_robotContainer.setLedState(LedState.DISABLED);
    }

    @Override
    public void robotPeriodic() {
        // Runs the scheduler, including default commands
        CommandScheduler.getInstance().run();
        m_robotContainer.updateTelemetry();
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.setLedState(LedState.AUTONOMOUS);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        m_robotContainer.setLedState(LedState.TELEOP);
        // Cancel autonomous when teleop starts
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void disabledInit() {
        // Optionally reset anything needed when disabled
        m_robotContainer.setLedState(LedState.DISABLED);
    }

    @Override
    public void testInit() {
        m_robotContainer.setLedState(LedState.TEST);
        // Cancel all running commands during test
        CommandScheduler.getInstance().cancelAll();
    }
}
