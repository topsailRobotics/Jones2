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

public class ArmSubsystem_Manual extends SubsystemBase {
  //set up shooter motors
  private final CANSparkMax m_rightarm = new CANSparkMax(ArmConstants.kRightArmCanId, MotorType.kBrushless);
  private final CANSparkMax m_leftarm = new CANSparkMax(ArmConstants.kLeftArmCanId, MotorType.kBrushless);
  
  private PIDController pid;
  
  private double targetPosition = ArmConstants.defaultPosition;
  
  private final AbsoluteEncoder m_rightarmencoder = m_rightarm.getAbsoluteEncoder();

  

 /** The shooter subsystem for the robot. */
 public ArmSubsystem_Manual() {
  m_leftarm.setInverted(true);
  //pid.enableContinuousInput(0, 1);
  // m_rightarm.enableSoftLimit(SoftLimitDirection.kForward, true);
  // m_rightarm.enableSoftLimit(SoftLimitDirection.kReverse, true);
  // m_leftarm.enableSoftLimit(SoftLimitDirection.kForward, true);
  // m_leftarm.enableSoftLimit(SoftLimitDirection.kReverse, true);
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

}

public double getTargetPosition() {
  return targetPosition;
}

@Override
public void periodic() {
// if (m_rightarmencoder.getPosition()<=4.4&&armSpeed<=0) {
//   armSpeed = 0
//  }
SmartDashboard.putNumber("ArmEncoder",m_rightarmencoder.getPosition());
}
public void armStop(){
  m_rightarm.set(0);
  m_leftarm.set(0);
}

public void armLowCommand(){
  if (m_rightarmencoder.getPosition()<226.5) {
    m_rightarm.set(0.2);
    m_leftarm.set(0.2);
  }
  else if(m_rightarmencoder.getPosition()==(225.5-226.5)) {
    m_rightarm.set(0);
    m_leftarm.set(0);
  }
  else if(m_rightarmencoder.getPosition()>225.5) {
    m_rightarm.set(-0.1);
    m_leftarm.set(-0.1);
    }
  }
  public void armDefaultCommand(){
    if (m_rightarmencoder.getPosition()<182) {
      m_rightarm.set(0.2);
      m_leftarm.set(0.2);
    }
    else if(m_rightarmencoder.getPosition()==(181-182)) {
      m_rightarm.set(0);
      m_leftarm.set(0);
    }
    else if(m_rightarmencoder.getPosition()>182) {
      m_rightarm.set(-0.2);
      m_leftarm.set(-0.2);
      }
    }
//default=180
//low=223
//high=270


}

