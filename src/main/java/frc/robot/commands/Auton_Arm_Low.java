// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class Auton_Arm_Low extends CommandBase 
{


  private final Arm m_arm;
  private Double armEncoder = 0.0;
  private boolean toggle;
  private int upEncoder =(int)(50.49689251)*2048*100/360; 
  private int downEncoder = 0; 

    /** Creates a new Arm. */
  public Auton_Arm_Low(Arm arm, boolean _toggle) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
     this.m_arm = arm;
    addRequirements(arm);
    this.toggle = _toggle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_arm.arm_init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    armEncoder = m_arm.arm_encoder();
    SmartDashboard.putNumber("Arm_Encoder", armEncoder);
  //  m_arm.arm_on(Constants.AutoConstants.AUTON_ARM_SPEED);

    if(toggle == true)
    {
      m_arm.arm_on(Constants.AutoConstants.AUTON_ARM_SPEED);
      if(armEncoder <= upEncoder)
       {
       m_arm.arm_off();
       }
    }
    else if(toggle == false)
    {
      m_arm.arm_on(-Constants.AutoConstants.AUTON_ARM_SPEED);
      if(armEncoder <= downEncoder)
       {
       m_arm.arm_off();
       }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_arm.arm_off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(toggle == true)
    {
      if(armEncoder >= upEncoder)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    else if(toggle == false)
    {
      if(armEncoder <= downEncoder)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    else
    {
      return false;
    }
    }
}
