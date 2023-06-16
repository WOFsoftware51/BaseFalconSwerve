package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.Auton_TeleopSwerve;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Nothing_Auto extends SequentialCommandGroup 
{
    public Nothing_Auto(Swerve s_Swerve)
    {
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0, 0),
                        new Translation2d(0, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, -0.5, new Rotation2d(0)),
                config);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            new Auton_TeleopSwerve(s_Swerve, 0.1, 0, 0, 2.3, 0, false)
        );
    }
}