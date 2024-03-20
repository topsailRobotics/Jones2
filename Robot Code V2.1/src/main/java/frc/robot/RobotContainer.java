// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();

  private final ShooterIntakeSubsystem m_robotShooterIntake = new ShooterIntakeSubsystem();

  private final ShooterSubsystem m_robotShooter = new ShooterSubsystem(m_robotShooterIntake);
  
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  
  public RobotContainer() {
    // Register Named Commands
    NamedCommands.registerCommand("shootBump", m_robotShooterIntake.runShooterIntake());
    
    //    NamedCommands.registerCommand("runIntake", MoveArmToDefault());
    //    NamedCommands.registerCommand("runShooter", ShooterSubsystem.runShooter());
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    m_robotIntake.setDefaultCommand(
        new RunCommand(
            () -> m_robotIntake.stopIntake(),
            m_robotIntake));

    m_robotShooterIntake.setDefaultCommand(
        new RunCommand(
            () -> m_robotShooterIntake.stopShooterIntake(),
            m_robotShooterIntake));

    m_robotShooter.setDefaultCommand(
        new RunCommand(
            () -> m_robotShooter.stopShooter(),
            m_robotShooter));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        //.whileTrue(new RunCommand(
            //() -> m_robotDrive.setX(),
            //m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
            .toggleOnTrue(new RunCommand(
                () -> m_robotIntake.runIntake(),
                m_robotIntake));
    
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
            .toggleOnTrue(new RunCommand(
                () -> m_robotShooterIntake.runShooterIntake(),
                m_robotShooterIntake));     
    
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .toggleOnTrue(new RunCommand(
            () -> m_robotShooter.runShooter(), 
            m_robotShooter));      

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
;
            
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
; 

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
            .onTrue(m_robotDrive.zeroHeadingCommand()); 
            

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    .toggleOnTrue(new RunCommand(
        () -> m_robotShooterIntake.runShooterIntakeReverse(),
        m_robotShooterIntake));  
  }

/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 public Command getAutonomousCommand() {
//return Commands.run(m_robotShooter::auto, m_robotShooter);

return null;

// PathPlannerPath path = PathPlannerPath.fromPathFile("defaultpath");
//return AutoBuilder.followPath(path);

 }
}
