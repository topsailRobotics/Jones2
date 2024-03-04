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
    // Sets the error tolerance to 5, and the error derivative tolerance to 10 per second
    pid.setTolerance(5, 10);

    // Returns true if the error is less than 5 units, and the
    // error derivative is less than 10 units
    pid.atSetpoint();
    
    // The integral gain term will never add or subtract more than 0.5 from
    // the total loop output
    pid.setIntegratorRange(-0.5, 0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void update() {
  m_innerarm.set(pid.calculate(encoder.getDistance(), setpoint));
  m_outerarm.set(pid.calculate(encoder.getDistance(), setpoint));
  }
}
