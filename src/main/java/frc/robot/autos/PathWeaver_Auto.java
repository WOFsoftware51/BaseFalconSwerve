//as
package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class PathWeaver_Auto extends SequentialCommandGroup 
{
    public PathWeaver_Auto(Swerve s_Swerve, Arm m_arm, Intake m_intake, Extend m_extend, Wrist m_wrist)
    {
        String trajectoryJSON = "paths/Cube_Pickup.wpilib.json";
        Trajectory trajectory = new Trajectory();
        
        try 
        {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } 
         catch (IOException e) 
         {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, e.getStackTrace());
         }

        var thetaController =
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



        addCommands(
            new InstantCommand(() -> s_Swerve.zeroGyro()),
          /*   new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
                new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH, m_wrist, Constants.WRIST_SCORE), 
                new Auton_Wait(100)),
            new Auton_Intake(m_intake, 20, false),
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, 0), 
                new ScoreMiddle(m_arm, 0, m_wrist, 0), 
                new Auton_Wait(100)),*/
            swerveControllerCommand
        );
    }
}