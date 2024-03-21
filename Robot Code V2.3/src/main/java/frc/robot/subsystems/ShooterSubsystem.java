// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.WheelConveyorConstants;

public class ShooterSubsystem extends SubsystemBase {
  public ShooterIntakeSubsystem m_IntakeSubsystem;
  //set up shooter motors
  public final CANSparkMax m_bottomshooter = new CANSparkMax(WheelConveyorConstants.kBottomShooterCanId, MotorType.kBrushless);
  public final CANSparkMax m_topshooter = new CANSparkMax(WheelConveyorConstants.kTopShooterCanId, MotorType.kBrushless);

  

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem(ShooterIntakeSubsystem intakeSubsystem) {
    m_bottomshooter.setInverted(true);
    m_IntakeSubsystem = intakeSubsystem;
  }

  public void auto(){
    runFastShooter();
    Timer.delay(0.5);
    m_IntakeSubsystem.runShooterIntake();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runFastShooterCommand() {
    return this.runOnce(() -> m_bottomshooter.setVoltage(6)).alongWith(this.runOnce(() -> m_topshooter.setVoltage(8)));
  }

  public Command runSlowShooterCommand() {
    return this.runOnce(() -> m_bottomshooter.setVoltage(6)).alongWith( this.runOnce(() -> m_topshooter.setVoltage(6)));
  }

  public Command stopShooterCommand() {
    return this.runOnce(() -> m_bottomshooter.setVoltage(8)).alongWith(this.runOnce(() -> m_topshooter.setVoltage(8)));
  }

public void runFastShooter() {
  m_bottomshooter.setVoltage(6);
  m_topshooter.setVoltage(6);
}
public void runFieldShooter() {
  m_bottomshooter.setVoltage(10);
  m_topshooter.setVoltage(10);
}
public void stopShooter() {
  m_bottomshooter.setVoltage(0);
  m_topshooter.setVoltage(0);
}
}
