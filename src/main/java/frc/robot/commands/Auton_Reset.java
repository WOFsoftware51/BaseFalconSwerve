// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Swerve;


public class Auton_Reset extends CommandBase 
{

  private final Arm m_arm;
  private final Wrist m_wrist;
  private final Extend m_extend;
  private final Swerve s_Swerve;
  private Boolean endCommand = false;
  private int count = 0;


  /** Creates a new Arm. */
  public Auton_Reset(Arm arm,  Wrist wrist, Extend extend, Swerve s_Swerve) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.m_arm = arm;
    addRequirements(m_arm);
    this.m_wrist = wrist;
    addRequirements(m_wrist);
    this.m_extend = extend;
    addRequirements(m_extend);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_arm.arm_init();
    m_wrist.wrist_init();
    m_extend.extend_init();
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    m_wrist.Wrist_Goto_Angle(0);
    m_arm.Arm_Goto_Angle(0);
    m_extend.Extend_Goto_Length(0);

    /* 
    s_Swerve.drive(
      new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
      0 * Constants.Swerve.maxAngularVelocity, 
      false, 
      true);
    */
    
    if (count < 3)
    {
      count = 0;
      endCommand = true;
    }
    else
    {
        count++;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_arm.arm_off();
    m_wrist.Wrist_Off();
    m_extend.extend_off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return endCommand;
  }
}