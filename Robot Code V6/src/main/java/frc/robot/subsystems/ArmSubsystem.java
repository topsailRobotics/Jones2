// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

public class ArmSubsystem extends SubsystemBase {
  //set up shooter motors
  private final CANSparkMax m_rightarm = new CANSparkMax(ArmConstants.kRightArmCanId, MotorType.kBrushless);
  private final CANSparkMax m_leftarm = new CANSparkMax(ArmConstants.kLeftArmCanId, MotorType.kBrushless);
  
  private PIDController pid;
  
  private double targetPosition = ArmConstants.defaultPosition;
  
  private final AbsoluteEncoder m_rightarmencoder = m_rightarm.getAbsoluteEncoder();

  

 /** The shooter subsystem for the robot. */
 public ArmSubsystem() {
  m_rightarm.setInverted(true);
  pid = new PIDController(ArmConstants.kP, ArmConstants.kI,ArmConstants.kD);
  pid.enableContinuousInput(0, 1);
  setTargetPosition(targetPosition);
  m_rightarm.enableSoftLimit(SoftLimitDirection.kForward, true);
  m_rightarm.enableSoftLimit(SoftLimitDirection.kReverse, true);
  m_leftarm.enableSoftLimit(SoftLimitDirection.kForward, true);
  m_leftarm.enableSoftLimit(SoftLimitDirection.kReverse, true);
}
public void setSoftLimit(){
 }

public void moveArm(double power) {
  m_rightarm.set(power);
  m_leftarm.set(power);
}


public double getPosition() {
  return m_rightarmencoder.getPosition();
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
double armSpeed = pid.calculate(getPosition());
// if (m_rightarmencoder.getPosition()<=4.4&&armSpeed<=0) {
//   armSpeed = 0;
//  }
  m_rightarm.set(armSpeed);
  m_leftarm.set(armSpeed);
SmartDashboard.putNumber("armSpeed",armSpeed);
SmartDashboard.putNumber("ArmEncoder",m_rightarmencoder.getPosition());
SmartDashboard.putData("armPID",pid);
}

}

