package frc.robot.autos;

import frc.robot.commands.Limelight_Auton;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Limelight_Auto extends SequentialCommandGroup 
{
  Swerve s_Swerve;
    public Limelight_Auto(Swerve swerve)
    {
      this.s_Swerve = swerve;
      addRequirements(swerve);

        addCommands(
          new Limelight_Auton(swerve, 0, 10)
        );
    }
}