// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

public class ArmSubsystem extends SubsystemBase {
  //set up shooter motors
  private final CANSparkMax m_innerarm = new CANSparkMax(ArmConstants.kInnerArmCanId, MotorType.kBrushless);
  private final CANSparkMax m_outerarm = new CANSparkMax(ArmConstants.kOuterArmCanId, MotorType.kBrushless);
  
  private PIDController pid;
  
  private final RelativeEncoder m_innerarmencoder = m_innerarm.getEncoder();
  
  private double targetPosition = ArmConstants.defaultPosition;

 /** The shooter subsystem for the robot. */
 public ArmSubsystem() {
  pid = new PIDController(ArmConstants.kP, ArmConstants.kI,ArmConstants.kD);
  pid.enableContinuousInput(0, 1);
  m_innerarm.restoreFactoryDefaults();
   
  setTargetPosition(targetPosition);
  setSoftLimit();
}
public void setSoftLimit(){
  m_innerarm.enableSoftLimit(SoftLimitDirection.kForward, false);
  m_innerarm.enableSoftLimit(SoftLimitDirection.kReverse, false);
  m_outerarm.enableSoftLimit(SoftLimitDirection.kForward, false);
  m_outerarm.enableSoftLimit(SoftLimitDirection.kReverse, false);
 }

public void moveArm(double power) {
  m_innerarm.set(power);
  m_outerarm.set(power);
}


public double getPosition() {
  return m_innerarmencoder.getPosition();
}

public void setTargetPosition(double position){
  targetPosition = position;
  pid.setSetpoint(targetPosition);
}

public double getTargetPosition() {
  return targetPosition;
}

@Override
public void periodic() {
  m_innerarm.set(pid.calculate(getPosition()));
  m_outerarm.set(pid.calculate(getPosition()));

}

}

