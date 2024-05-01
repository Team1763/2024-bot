package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

  private PWMTalonSRX m_topMotor;
  private PWMTalonSRX m_bottomMotor;

  public boolean m_launcherRunning;

  private Timer timer;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    // create two new SPARK MAXs and configure them
    m_topMotor = new PWMTalonSRX(0);
    m_topMotor.setInverted(false);

    m_bottomMotor =
        new PWMTalonSRX(1);
    m_bottomMotor.setInverted(false);

    timer = new Timer();
    
  }

  /**
   * Turns the launcher on. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void runLauncher() {
    m_launcherRunning = true;
  }

  /**
   * Turns the launcher off. Can be run once and the launcher will stay running or run continuously
   * in a {@code RunCommand}.
   */
  public void stopLauncher() {
    m_launcherRunning = false;
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not
    if (m_launcherRunning) {
     // m_topMotor.set(Constants.Launcher.kTopPower);
     // m_bottomMotor.set(Constants.Launcher.kBottomPower);
    } else {
     // m_topMotor.set(0.0);
     // m_bottomMotor.set(0.0);
    }
  }

public void runTopMotor(double speed) {
  m_topMotor.set(speed);
}

public void runBottomMotor(double speed) {
  m_bottomMotor.set(speed);
}
} 