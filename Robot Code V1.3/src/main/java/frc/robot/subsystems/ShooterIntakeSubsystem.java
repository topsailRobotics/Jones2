// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.WheelConveyorConstants;

public class ShooterIntakeSubsystem extends SubsystemBase {

  //set up shooter motors
  private final CANSparkMax m_shooterintake = new CANSparkMax(WheelConveyorConstants.kShooterIntakeCanId, MotorType.kBrushless);
  

  /** Creates a new IntakeSubsystem. */
  public ShooterIntakeSubsystem() {
    m_shooterintake.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void stopShooterIntake() {
    m_shooterintake.setVoltage(0);
  }

  public void runShooterIntake() {
    m_shooterintake.setVoltage(12);
  }

}
