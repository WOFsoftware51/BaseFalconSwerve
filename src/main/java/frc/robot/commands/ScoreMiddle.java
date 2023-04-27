// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class ScoreMiddle extends CommandBase 
{

  private final Arm m_arm;
  private final Wrist m_wrist;
  private double armEncoder = 0.0;
  private int count = 0;


  private double armTarget;
  private double wristTarget;


  /** Creates a new Arm. */
  public ScoreMiddle(Arm arm, double armAngle, Wrist wrist, double wristAngle) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = arm;
    addRequirements(arm);
    this.m_wrist = wrist;
    addRequirements(wrist);
    armTarget = armAngle;
    wristTarget = wristAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    count = 0;
    m_arm.arm_init();
    m_wrist.wrist_init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(count > 2 && (armEncoder < -20 || armEncoder > 20))
    {
      m_arm.updateEncoder();
      count = 0;
    }
    else
    {
      count++;
    }
    
    m_wrist.Wrist_Goto_Angle(wristTarget);
    m_arm.Arm_Goto_Angle(armTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_arm.arm_off();
    m_wrist.Wrist_Off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
