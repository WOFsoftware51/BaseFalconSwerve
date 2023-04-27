// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class Wrist_Command extends CommandBase 
{

  private final DoubleSupplier m_translationXSupplier;
  private final Wrist m_wrist;
  private double wristEncoder = 0.0;
  private double wristCANCoder = 0.0;

  /** Creates a new Arm. */
  public Wrist_Command(DoubleSupplier translationXSupplier, Wrist wrist) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_translationXSupplier = translationXSupplier;
    this.m_wrist = wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_wrist.wrist_init();
    // m_arm.arm_resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_wrist.Wrist_On(m_translationXSupplier.getAsDouble());

    wristEncoder = m_wrist.wrist_encoder();
    wristCANCoder = m_wrist.wrist_CANCoder();
    SmartDashboard.putNumber("wrist Encoder", wristEncoder);
    SmartDashboard.putNumber("wrist CANCoder", wristCANCoder);
    SmartDashboard.putNumber("wrist input", m_translationXSupplier.getAsDouble());


  
     if(wristEncoder > 105)
    {
      if(m_translationXSupplier.getAsDouble() < 0)
      {
        m_wrist.Wrist_On(m_translationXSupplier.getAsDouble()*0.5);
      }
      else
      {
        m_wrist.Wrist_Off();
      }
    }
    else if(wristEncoder < -105)
    {
      if(m_translationXSupplier.getAsDouble() > 0)
      {
        m_wrist.Wrist_On(m_translationXSupplier.getAsDouble()*0.5);
      }
      else
      {
        m_wrist.Wrist_Off();
      }
    }
    else
    {
      m_wrist.Wrist_On(m_translationXSupplier.getAsDouble()*0.5);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_wrist.Wrist_Off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
