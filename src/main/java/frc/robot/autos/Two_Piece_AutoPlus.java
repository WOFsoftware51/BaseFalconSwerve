package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Global_Variables;
import frc.robot.commands.Auton_Arm_Extend;
import frc.robot.commands.Auton_Intake;
import frc.robot.commands.Auton_Intake_Piece;
import frc.robot.commands.Auton_Wait;
import frc.robot.commands.ScoreMiddle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extend;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Two_Piece_AutoPlus extends SequentialCommandGroup 
{
    private final Arm m_arm;
    private final Wrist m_wrist;
    private final Intake m_intake;
    private final Extend m_extend;
    private final Swerve s_Swerve;
    
    public Two_Piece_AutoPlus(Swerve swerve, Extend extend, Arm arm, Wrist wrist, Intake intake)
    {
        this.m_arm = arm;
        addRequirements(arm);
        this.m_wrist = wrist;
        addRequirements(wrist);
        this.m_extend = extend;
        addRequirements(extend);
        this.m_intake = intake;
        addRequirements(intake);
        this.s_Swerve = swerve;
        addRequirements(swerve);
        PathPlannerTrajectory Path1 = PathPlanner.loadPath("Cube_Pickup", new PathConstraints(4, 3));
        PathPlannerTrajectory Path2 = PathPlanner.loadPath("Cone_Pickup", new PathConstraints(4, 3));
        PathPlannerState exampleState = (PathPlannerState) Path1.sample(1.2);
        HashMap<String, Command> eventMap = new HashMap<>();
        HashMap<String, Command> eventMap2 = new HashMap<>();


        eventMap.put("IntakeDown", new PrintCommand("Yay"));
 
        FollowPathWithEvents command1 = new FollowPathWithEvents(
            s_Swerve.followTrajectoryCommand(Path1, true), 
            Path1.getMarkers(), 
            eventMap
            );

        FollowPathWithEvents command2 = new FollowPathWithEvents(
            s_Swerve.followTrajectoryCommand(Path2, true), 
            Path2.getMarkers(), 
            eventMap2
            );


        addCommands(
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
                new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH, m_wrist, Constants.WRIST_SCORE),
                new Auton_Wait(100)),
                new ParallelRaceGroup(
                    new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
                    new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH, m_wrist, Constants.WRIST_SCORE),
                    new Auton_Intake(intake, 20, false)),
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, 0), 
                new ScoreMiddle(m_arm, 0, m_wrist, 0), 
                new Auton_Wait(60)),
            new ParallelCommandGroup(
                command1 ,
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new Auton_Arm_Extend(m_extend,  0), 
                        new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CUBE, m_wrist, Constants.WRIST_PICKUP_CUBE), 
                        new Auton_Intake_Piece(intake, 150, true)),
                    new ParallelRaceGroup(
                        new Auton_Arm_Extend(m_extend,  0), 
                        new ScoreMiddle(m_arm, 0, m_wrist, 0), 
                        new Auton_Intake(intake, 100, true))
                    )
                
            ),
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, Constants.EXTEND_SCORE_HIGH), 
                new ScoreMiddle(m_arm, Constants.ARM_SCORE_HIGH, m_wrist, Constants.WRIST_SCORE), 
                new Auton_Wait(100)), 
            new Auton_Intake(intake, 20, false),
            new ParallelRaceGroup(
                new Auton_Arm_Extend(m_extend, 0), 
                new ScoreMiddle(m_arm, 0, m_wrist, 0), 
                new Auton_Wait(60)),
            new ParallelCommandGroup(
                command2,
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new Auton_Arm_Extend(m_extend,  0), 
                        new ScoreMiddle(m_arm, Constants.ARM_PICKUP_CUBE, m_wrist, Constants.WRIST_PICKUP_CUBE), 
                        new Auton_Intake_Piece(intake, 150, true)),
                    new ParallelRaceGroup(
                        new Auton_Arm_Extend(m_extend,  0), 
                        new ScoreMiddle(m_arm, 0, m_wrist, 0), 
                        new Auton_Intake(intake, 100, true))
                    )
            )
        );
    }
}