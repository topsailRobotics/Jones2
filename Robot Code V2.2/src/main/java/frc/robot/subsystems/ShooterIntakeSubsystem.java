// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
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

  public Command runShooterIntake() {
    return this.runOnce(() -> m_shooterintake.setVoltage(4));
  }
  public void runShooterIntakeReverse() {
    m_shooterintake.setVoltage(-2);
  }
public void runShooterIntakeIfSensor() {
    try (DigitalInput m_sensor = new DigitalInput(0)) {
      if (m_sensor.get()){
          m_shooterintake.setVoltage(4);
      } else {
       m_shooterintake.setVoltage(0);
        int timer = 0;
        timer++;
        if (timer<=300) {
          m_shooterintake.setVoltage(-2);
        } 
      }
    }
  }
}
