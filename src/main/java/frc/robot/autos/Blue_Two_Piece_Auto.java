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
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.Auton_Arm_Extend;
import frc.robot.commands.Auton_Intake;
import frc.robot.commands.Auton_Wait;
import frc.robot.commands.ScoreMiddle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Auton_Subsystem;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift_Pneumatic;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;


// NOTE:  Consider using GetAbsolutePosition this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Blue_Two_Piece_Auto extends SequentialCommandGroup 
{
  /** Creates a new One_Ball_Auto. */
  public Blue_Two_Piece_Auto(Lift_Pneumatic m_lift, Swerve s_Swerve, Auton_Subsystem m_auto, Arm m_arm, Intake m_intake, Extend m_extend, Wrist m_wrist) 
   {

    TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(true);
  
    // An example trajectory to follow.  All units in meters.
    Trajectory trajectory =
    TrajectoryGenerator.generateTrajectory(
     // Start at the origin facing the +X direction
     new Pose2d(0, 0, new Rotation2d(0)),
     // Pass through these two interior waypoints, making an 's' curve path
     List.of(new Translation2d(-3.42245, 0)),
     // End 3 meters straight ahead of where we started, facing forward
     new Pose2d(-4.8768, -0.4064, new Rotation2d(0)),
     config
     );

     Trajectory trajectory2 =
     TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(-4.8768, -0.4064, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(-3.42245, 0)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(0, -0.4064, new Rotation2d(0)),
      config
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
      new ParallelRaceGroup(
        new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH),
        new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH, m_wrist, Constants.WRIST_SCORE),
        new Auton_Wait(100)
      ),
      new Auton_Intake(m_intake, 20, false),
      new ParallelRaceGroup(
        new Auton_Arm_Extend(m_extend, 0),
        new ScoreMiddle(m_arm, 0, m_wrist, 0),
        new Auton_Wait(100)
      ),
      new ParallelCommandGroup(
        swerveControllerCommand, 
        new ParallelRaceGroup(
          new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CUBE, m_wrist, Constants.WRIST_PICKUP_CUBE), 
          new Auton_Wait(100)
        )
      ),
      new Auton_Intake(m_intake, 100, true),
      swerveControllerCommand2,
      new ParallelRaceGroup(
        new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
        new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH, m_wrist, Constants.WRIST_SCORE), 
        new Auton_Wait(100)
      ),
      new Auton_Intake(m_intake, 100, false)
    );
  }
}