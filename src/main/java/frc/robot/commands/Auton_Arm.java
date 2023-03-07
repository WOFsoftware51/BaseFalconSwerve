// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;

public class Auton_Arm extends CommandBase 
{
  private final Arm m_arm;
  private Double armEncoder = 0.0;
  private boolean toggle;
  private double upEncoder = Conversions.degreesToFalcon(Constants.ARM_SCORE_HIGH, Constants.ARM_CONVERSION); 
  private double downEncoder = Conversions.degreesToFalcon(Constants.ARM_PICKUP_CUBE, Constants.ARM_CONVERSION);

    /** Creates a new Arm. */
  public Auton_Arm(Arm m_arm2, boolean _toggle) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
     this.m_arm = m_arm2;
    addRequirements(m_arm2);
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
      if(armEncoder > upEncoder)
      {
        m_arm.arm_off();
      }
      else
      {
        m_arm.arm_on(-Constants.AutoConstants.AUTON_ARM_SPEED);;
      }
    }
    else if(toggle == false)
      {
        if(armEncoder < downEncoder)
        {
          m_arm.arm_off();
        }
        else
        {
          m_arm.arm_on(Constants.AutoConstants.AUTON_ARM_SPEED);;
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
