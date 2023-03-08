package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class Auton_TeleopSwerve extends CommandBase 
{    
    private Swerve s_Swerve;    
    private double translation;
    private double strafe;
    private double rotation;
    private int counter;
    private int time = 0;
    private  boolean endCommand = false;

    public Auton_TeleopSwerve(Swerve s_Swerve, double translation, double strafe, double rotation, int time) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.time = time;

    }

    @Override
    public void initialize() 
    {
      time = 0;
      endCommand = false;
    }

    @Override
    public void execute() 
    {

      if(counter > time)
      {
        endCommand =  true;
        time = 0;
      }
        /* Drive */
        s_Swerve.drive(
            new Translation2d(translation, strafe).times(Constants.Swerve.maxSpeed), 
            rotation * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
        counter++;
    }

    @Override
    public boolean isFinished() 
    { 
      return endCommand;

    }  
}