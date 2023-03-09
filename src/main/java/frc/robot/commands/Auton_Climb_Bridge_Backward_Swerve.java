package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class Auton_Climb_Bridge_Backward_Swerve extends CommandBase 
{    
    private Swerve s_Swerve;    
    private double translation;

    private double pitch;
    private int timeout;
    private Boolean endCommand = false;
    private int count = 0;
    
    public Auton_Climb_Bridge_Backward_Swerve(Swerve swerve, double translation, int timeout) {
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
    //  translation = translation *s_Swerve.SpeedModifier;
      if(pitch > 7 && timeout > count)
      {
        count++;
        /* Drive */
        s_Swerve.drive(
          new Translation2d(translation, 0).times(Constants.Swerve.maxSpeed), 
          0 * Constants.Swerve.maxAngularVelocity, 
          false, 
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