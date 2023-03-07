// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class ScoreDefault extends CommandBase 
{

  private final Arm m_arm;
  private final Wrist m_wrist;
  private boolean toggle;
  private double angleArm;
  private double angleWrist;
  private int sign;

  /** Creates a new Arm. */
  public ScoreDefault(Arm arm, Wrist wrist, boolean _signToggle, double _angleArm, double _angleWrist) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = arm;
    addRequirements(arm);
    this.m_wrist = wrist;
    addRequirements(wrist);
    this.toggle = _signToggle;
    this.angleArm = _angleArm;
    this.angleWrist = _angleWrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_arm.arm_init();
    m_wrist.wrist_init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(toggle)
    {
      sign = 1;
    }
    else if(toggle==false)
    {
      sign = -1;
    }

    m_arm.arm_angle(Conversions.degreesToFalcon(angleArm*sign, Constants.ARM_GEAR_RATIO));
    m_arm.arm_angle(Conversions.degreesToFalcon(angleArm*sign, Constants.ARM_GEAR_RATIO));
   // m_wrist.Wrist_Score(Conversions.degreesToFalcon(angleWrist*sign, Constants.WRIST_GEAR_RATIO));
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
