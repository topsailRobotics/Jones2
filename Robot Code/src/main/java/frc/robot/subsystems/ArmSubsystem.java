// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class ArmSubsystem extends SubsystemBase {
  // Creates a PIDController with gains kP, kI, and kD
  PIDController pid = new PIDController(kP, kI, kD);
  
  //set up shooter motors
  private final CANSparkMax m_innerarm = new CANSparkMax(ArmConstants.kInnerArmCanId, MotorType.kBrushless);
  private final CANSparkMax m_outerarm = new CANSparkMax(ArmConstants.kOuterArmCanId, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public ArmSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
