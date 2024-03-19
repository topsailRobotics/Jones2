// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.WheelConveyorConstants;

public class ShooterSubsystem extends SubsystemBase {

  //set up shooter motors
  private final CANSparkMax m_bottomshooter = new CANSparkMax(WheelConveyorConstants.kBottomShooterCanId, MotorType.kBrushless);
  private final CANSparkMax m_topshooter = new CANSparkMax(WheelConveyorConstants.kTopShooterCanId, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
    m_bottomshooter.setInverted(true);
    m_topshooter.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void runShooter() {
  m_bottomshooter.setVoltage(12);
  m_topshooter.setVoltage(12);
}

public void stopShooter() {
  m_bottomshooter.setVoltage(0);
  m_topshooter.setVoltage(0);
}
}
