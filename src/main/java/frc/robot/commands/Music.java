// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;

public class Music extends CommandBase 
{
  private final Swerve s_swerve;
  // private final Arm m_arm;
  // private final Intake m_intake;
  // private final Extend m_extend;
  // private final Wrist m_wrist;
  private final boolean On;
  
  public Music(Swerve swerve, boolean on)//, Arm arm, Wrist wrist, Intake intake, Extend extend, boolean on)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_swerve = swerve;
    addRequirements(s_swerve);
    // this.m_arm = arm;
    // addRequirements(m_arm);
    // this.m_intake = intake;
    // addRequirements(m_intake);
    // this.m_extend = extend;
    // addRequirements(m_extend);
    // this.m_wrist = wrist;
    // addRequirements(m_wrist);
    this.On = on;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // s_swerve.addToInstruments(m_arm.returnArmMotors());
    // s_swerve.addToInstruments(m_intake.returnArmMotors());
    // s_swerve.addToInstruments(m_extend.returnArmMotors());
    // s_swerve.addToInstruments(m_wrist.returnArmMotors());

    s_swerve.music_init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      if(On)
      {
        s_swerve.play_music();
      }
      else
      {
        s_swerve.stop_music();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
   // m_arm.arm_off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
      return false;
  }
}
