package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class runLauncher extends Command {
    private double autonomousStartTime;
    private LauncherSubsystem m_Launcher;
    private boolean commandStateFinished = false;
    private DoubleSupplier speed;

    private Timer timer;

    public runLauncher(LauncherSubsystem m_launcher, DoubleSupplier speed) {
        addRequirements(m_launcher);
        this.m_Launcher = m_launcher;
        this.speed = speed;

        timer = new Timer();
    }

    @Override
    public void initialize() {
        // Runs once on start
        timer.reset();

    }

    @Override
    public void execute() {
        if (speed.getAsDouble() == 0) {
            timer.stop();
            timer.reset();
        } else {
            timer.start();
        }

        m_Launcher.runTopMotor(speed.getAsDouble());
        if (timer.get() >= 1 || speed.getAsDouble() < 0) {
            m_Launcher.runBottomMotor(speed.getAsDouble() < 0 ? speed.getAsDouble() : 1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
        m_Launcher.runTopMotor(0);
        m_Launcher.runBottomMotor(0);
    }

    @Override
    public boolean isFinished() {
        return commandStateFinished;
        // Whether or not the command is finished
    }
}