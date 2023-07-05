// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Limelight_Change_Pipeline extends CommandBase {
  /** Creates a new Limelight_Pipeline_0. */
  
  private final Swerve s_swerve;
  private double count = 0;
  private boolean endCommand = false;

  public Limelight_Change_Pipeline(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_swerve = swerve;
    addRequirements(s_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    s_swerve.cameraPipeline();
    count++;
    if(count > 0){
      endCommand = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
    
  }
}
