// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;


import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.Auton_Arm;
import frc.robot.commands.Auton_Arm_Extend;
import frc.robot.commands.Auton_Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift_Pneumatic;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using GetAbsolutePosition this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class One_Piece_Auto extends SequentialCommandGroup 
{
  /** Creates a new One_Ball_Auto. */
  public One_Piece_Auto(Lift_Pneumatic m_lift, Swerve s_Swerve, Auton_Subsystem m_auto, Extend m_extend, Intake m_intake, Arm m_arm) 
   {
    // An example trajectory to follow.  All units in meters.
    Trajectory trajectory =
    TrajectoryGenerator.generateTrajectory(
     // Start at the origin facing the +X direction
     new Pose2d(0, 0, new Rotation2d(0)),
     // Pass through these two interior waypoints, making an 's' curve path
     List.of(new Translation2d(0, 0),
             new Translation2d(0, 0)),
     // End 3 meters straight ahead of where we started, facing forward
     new Pose2d(-5.6896, 0, new Rotation2d(Math.PI)),
     m_auto.config
     );

     Trajectory trajectory2 =
     TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(-5.6896, 0, new Rotation2d(Math.PI)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(0, 0),
              new Translation2d(0, 0)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(0, 0, new Rotation2d(0)),
      m_auto.config
      );

      
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);


    SwerveControllerCommand swerveControllerCommand2 =
        new SwerveControllerCommand(
            trajectory2,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
         //new Auton_Arm_Extend(m_extend, true),
      new Auton_Arm_Extend(m_extend, -50000)
      
     // new Auton_Arm_Extend(m_extend, false)
       );
  }
}
