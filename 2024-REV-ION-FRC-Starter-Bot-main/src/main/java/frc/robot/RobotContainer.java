// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.basicAuto;
import frc.robot.commands.runLauncher;
// import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.utils.GamepadUtils;
import java.util.List;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private final ArmSubsystem m_arm = new ArmSubsystem();
  // private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  SendableChooser<Command> m_Chooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //m_Chooser.addOption("basicAuto", new basicAuto(m_robotDrive));
    m_Chooser.setDefaultOption("basicAuto",  new basicAuto(m_robotDrive));
    SmartDashboard.putData("Auto chooser", m_Chooser);

    
    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.resetOdometry(new Pose2d());

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -GamepadUtils.squareInput(
                        m_driverController.getY(), OIConstants.kDriveDeadband) * -((m_driverController.getThrottle() - 1) / 2),
                    -GamepadUtils.squareInput(
                        m_driverController.getX(), OIConstants.kDriveDeadband) * -((m_driverController.getThrottle() - 1) / 2),
                    -GamepadUtils.squareInput(
                        m_driverController.getZ(), OIConstants.kDriveDeadband) * -((m_driverController.getThrottle() - 1) / 2),
                    true,
                    false),
            m_robotDrive));
    
    m_launcher.setDefaultCommand(new runLauncher(m_launcher, () -> 0));

    // set the arm subsystem to run the "runAutomatic" function continuously when no other command
    // is running
    // m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));

    // set the intake to stop (0 power) when no other command is running
    // m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));

    // configure the launcher to stop when no other command is running
    // m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // button to put swerve modules in an "x" configuration to hold position
    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // set up arm preset positions
    /*
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));
    new Trigger(
            () ->
                m_driverController.getLeftTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition)));

    // intake controls (run while button is held down, run retract command once when the button is
    // released)
    new Trigger(
            () ->
                m_driverController.getRightTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(new RunCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower), m_intake))
        .onFalse(m_intake.retract());

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> m_intake.setPower(-1.0)));

    */

    new JoystickButton(m_driverController, 1)
        .whileTrue(new runLauncher(m_launcher, () -> 1));

        //intake controls (button to spin launcher in reverse, to intake note)
    new JoystickButton(m_driverController, 2)
        .whileTrue(new runLauncher(m_launcher, () -> -0.5));
    
    new JoystickButton(m_driverController, 7)
        .whileTrue(new runLauncher(m_launcher, () -> 0.2));

    new JoystickButton(m_driverController, 5)
        .onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));
    
    /*

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(m_intake.feedLauncher(m_launcher));

    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new basicAuto(m_robotDrive);
  }
}
