// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extend;

public class Arm_Extend extends CommandBase 
{
  private final Extend m_extend;
  private double extendEncoder = 0.0;
  private double extendSpeed = 0.0;
  private Boolean in;
  /** Creates a new Arm_extend. 
   * @param extend */
  public Arm_Extend(Extend extend, Boolean _in) 
  { 
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_extend = extend;
    addRequirements(extend);
    this.in = _in;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
      m_extend.extend_init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    extendEncoder = m_extend.extend_encoder();
    extendSpeed = m_extend.extend_speed();
    SmartDashboard.putNumber("Extend_Encoder", extendEncoder);
    SmartDashboard.putNumber("Extend_Speed", extendSpeed);
    
    if(in)
    {
      m_extend.extend_on();
    }
    else if(in==false)
    {
      m_extend.extend_reverse();
    }

    /*if(armEncoder > 30000)
    {
      if(m_translationYSupplier.getAsDouble() < 0)
      {
        m_arm.arm_extend(m_translationYSupplier.getAsDouble());
      }
      else
      {
        m_arm.extend_off();
      }
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_extend.extend_off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
