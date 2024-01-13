// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/** Add your docs here. */
public class Global_Variables 
{
  public static double robot_direction = 1;
  public static double robot_directionY = 1;
  public static boolean left_bumper = false;
  public static boolean right_bumper = false;

  public static boolean left_bumper_boost = false;
  public static boolean right_bumper_boost = false;

  
  public static boolean have_game_piece = false;

  public static SendableChooser<Double> a_chooser_Arm = new SendableChooser<>();

  public static double arm_Angle;
  public static double extend_Position;
  public static double wrist_Angle;
  //Not using m_music at the moment
  // public static Orchestra m_music;

  public static SendableChooser<String> song = new SendableChooser<>();
  public static SendableChooser<Boolean> songBool = new SendableChooser<>();
  

}
