package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.LauncherSubsystem;



public class basicAuto extends Command {
    private double autonomousStartTime;
    private DriveSubsystem m_robotDrive;
    private boolean commandStateFinished = false;
//private final LauncherSubsystem m_launcher = new LauncherSubsystem();
    public basicAuto(DriveSubsystem m_robotDrive) {
        addRequirements(m_robotDrive);
        this.m_robotDrive = m_robotDrive;
    }
    @Override
    public void initialize() {
        // Runs once on start  
        autonomousStartTime = Timer.getFPGATimestamp();
 
    }

    @Override
    public void execute() {
        
        double elapsedTime = Timer.getFPGATimestamp() - autonomousStartTime;
        // Runs repeatedly after initialization
        

        if (elapsedTime < 3.0) {
           
            //new runLauncher(m_launcher, () -> 1);
            m_robotDrive.drive(0.250,
                    0.025,
                    0,
                    true,
                    false); 
        }
        else if (elapsedTime < 6.0){
            //Next item to implement If it is to wait write code to make it "Stop" or
            //Zero out the Drive train.
                m_robotDrive.drive(0,
                    0,
                    0,
                    true,
                    false);
        }
         else {
           
        }
    }


    @Override
    public void end(boolean interrupted) {
        // Runs when ended/cancelled
    }

    @Override
    public boolean isFinished() {
        return commandStateFinished;
        // Whether or not the command is finished
    }
}