// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lift_Pneumatic;

public class Lift_Pneumatic_On_Command extends CommandBase {
  /** Creates a new Arm_Pnematic_Command. */
  Lift_Pneumatic m_lift;
  public Lift_Pneumatic_On_Command(Lift_Pneumatic lift) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lift);
    this.m_lift = lift;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_lift.Arm_Pnematics_On();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
