// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Global_Variables;
//import frc.robot.subsystems.CANdle;
import frc.robot.subsystems.Intake;


public class Intake_Reverse_Command extends CommandBase 
{
  
  private final Intake m_intake;
  /** Creates a new Intake. */
  // private final CANdle m_candle;
  

  /** Creates a new Intake. */
  public Intake_Reverse_Command(Intake intake) 
  {
    this.m_intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
   // m_candle.CANdle_Intake(lednum);
   m_intake.intake_init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    m_intake.Intake_Reverse();
    Global_Variables.have_game_piece = false;  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_intake.Intake_Off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
