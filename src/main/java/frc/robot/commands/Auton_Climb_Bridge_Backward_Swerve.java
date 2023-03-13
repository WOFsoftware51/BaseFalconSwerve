package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class Auton_Climb_Bridge_Backward_Swerve extends CommandBase 
{    
    private Swerve s_Swerve;    
    private double translation;
    private double strafe;
    private double rotation;
    private double pitch;
    private int timeout;
    private Boolean endCommand = false;
    private int count = 0;
    
    public Auton_Climb_Bridge_Backward_Swerve(Swerve s_Swerve, double translation, double strafe, double rotation, int timeout) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
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
      if(pitch > 8 && timeout > count)  //-6
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