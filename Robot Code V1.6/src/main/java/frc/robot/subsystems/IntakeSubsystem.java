// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.WheelConveyorConstants;

public class IntakeSubsystem extends SubsystemBase {

  //set up instake motor
  private final CANSparkMax m_intake = new CANSparkMax(WheelConveyorConstants.kIntakeCanId, MotorType.kBrushless);


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void stopIntake() {
    m_intake.setVoltage(0);
  }

  public void runIntake() {
    m_intake.setVoltage(12);
  }
public void runIntakeIfSensor() {
    try (DigitalInput m_sensor = new DigitalInput(0)) {
      if (m_sensor.get()){
          m_intake.setVoltage(12);
      } else {
          m_intake.setVoltage(0);
      }
    }
  }
}
