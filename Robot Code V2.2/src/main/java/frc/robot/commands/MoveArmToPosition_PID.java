// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem_PID;

public class MoveArmToPosition_PID extends Command {
  private ArmSubsystem_PID arm;
  private double targetPosition;
  /** Creates a new MoveArmToPostion. */
  public MoveArmToPosition_PID(ArmSubsystem_PID arm, double targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.arm=arm;
    this.targetPosition=targetPosition;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  arm.setTargetPosition(targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}