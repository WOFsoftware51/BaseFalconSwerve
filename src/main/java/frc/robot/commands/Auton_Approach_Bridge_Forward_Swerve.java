package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class Auton_Approach_Bridge_Forward_Swerve extends CommandBase 
{    
    private Swerve s_Swerve;    
    private double translation;
    private double pitch;
    private int timeout;
    private Boolean endCommand = false;
    private int count = 0;
    
    public Auton_Approach_Bridge_Forward_Swerve(Swerve swerve, double translation, int timeout) {
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
      pitch = s_Swerve.getPitch();

      if(pitch > -9 && timeout > count)
      {
       // translation = translation *s_Swerve.SpeedModifier;
        count++;
        /* Drive */
        s_Swerve.drive
        (
          new Translation2d(translation, 0).times(Constants.Swerve.maxSpeed), 0.0, false, true
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