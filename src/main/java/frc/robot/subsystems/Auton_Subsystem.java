// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Auton_Subsystem extends SubsystemBase 
{
  Swerve s_Swerve;
  public TrajectoryConfig config =
  new TrajectoryConfig(
          Constants.AutoConstants.kMaxSpeedMetersPerSecond,
          Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          .setKinematics(Constants.Swerve.swerveKinematics).setReversed(true);

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
