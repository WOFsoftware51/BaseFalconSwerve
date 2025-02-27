package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class Auton_Approach_Flat_Swerve extends CommandBase 
{    
    private Swerve s_Swerve;    
    private double translation;
    private double pitch;
    private int timeout;
    private double turn_error;
    private double rotationPercent;
    private Boolean endCommand = false;
    private int count = 0;
    
    public Auton_Approach_Flat_Swerve(Swerve swerve, double translation, int timeout) {
        this.s_Swerve = swerve;
        addRequirements(s_Swerve);

        this.translation = translation;
        this.timeout = timeout;
    }

    @Override
    public void initialize() 
    {
      pitch = s_Swerve.getPitch();
      endCommand = false;
      count = 0;
    }

    @Override
    public void execute() 
    {


      turn_error = 0 - s_Swerve.yaw();
 
      if(turn_error<-30)
      {
        rotationPercent = 0.3;
      }
        else if(turn_error<-10)
      {
        rotationPercent = 0.15;
      }
        else if(turn_error<-1)
      {
        rotationPercent = 0.05;
      }
        else if(turn_error>30)
      {
        rotationPercent = -0.3;
      }
        else if(turn_error>10)
      {
        rotationPercent = -0.15;
      }
        else if(turn_error>1)
      {
        rotationPercent = -0.05;
      }
        else
      {
        rotationPercent = 0.0;
      }
      pitch = s_Swerve.getPitch();

      if(pitch > 2 && timeout > count)
      {
        count++;
        /* Drive */
        s_Swerve.drive
        (
          new Translation2d(translation, 0).times(Constants.Swerve.maxSpeed),
          rotationPercent * Constants.Swerve.maxAngularVelocity, 
          true, 
           true
        );
      }

      else
      {
        endCommand = true;
      }
      
    }

    public void end(boolean interrupted) 
  {
    s_Swerve.drive(
      new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
      0 * Constants.Swerve.maxAngularVelocity, 
      false, 
      true
    ); 
  }
    @Override
    public boolean isFinished() 
    { 
      return endCommand;
    }
 
}