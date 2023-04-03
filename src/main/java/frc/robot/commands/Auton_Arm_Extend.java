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
import frc.robot.subsystems.Extend;

public class Auton_Arm_Extend extends CommandBase 
{


   private final Extend m_extend;
  private double extendEncoder = 0.0;
  private double extendSpeed = 0.0;
  private double value = 0.0;


      /** Creates a new Arm. */
  public Auton_Arm_Extend(Extend extend, double value) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
     this.m_extend = extend;
    addRequirements(extend);
    this.value = value;
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
   // SmartDashboard.putNumber("Extend_Encoder", armEncoder);
    extendSpeed = m_extend.extend_speed();
    SmartDashboard.putNumber("Extend_Speed", extendSpeed);
    m_extend.Extend_Goto_Length(value);

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
