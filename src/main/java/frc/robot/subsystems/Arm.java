// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Global_Variables;

import com.ctre.phoenix.sensors.CANCoder;
/*Assignments:
 * Add deaband to the field centric arm so that there is less of a problem around 180 yaw
 * add constant for one side of the robot
*/

public class Arm extends SubsystemBase 
{
  private final TalonFX _arm = new TalonFX(Constants.Arm_Motor, Constants.CANIVORE_NAME );
  private final TalonFX _arm2 = new TalonFX(Constants.Arm_Motor_Slave, Constants.CANIVORE_NAME );
  private CANCoder armCANCoder = new CANCoder(Constants.Arm_CANCoder, Constants.CANIVORE_NAME);

  /** Creates a new Arm. */
  public void arm_init()
  {
 _arm.setNeutralMode(NeutralMode.Brake);
 _arm.setInverted(false);
 //_arm.configSupplyCurrentLimit(30);
 if(arm_CANCoder()!=0)
  {
    _arm.setSelectedSensorPosition((Constants.ARM_OFFSET-armCANCoder.getAbsolutePosition())*Constants.ARM_CONVERSION);  //  angle*2048*100(gear ratio)/360
  }
  else
  {
    _arm.setSelectedSensorPosition(0);  //  angle*2048*100(gear ratio)/360
  }

 _arm.config_kF(0, 0.035 , 30);
 _arm.config_kP(0, 0.015, 30);   //.0.15
 _arm.config_kI(0, 0.00, 30);
 _arm.config_kD(0, 0.0, 30);

 _arm.configMotionCruiseVelocity(20000);
 _arm.configMotionAcceleration(20000);
 _arm.configMotionSCurveStrength(6);

 _arm.setSensorPhase(true);
 _arm.configForwardSoftLimitEnable(false);
 _arm.configForwardSoftLimitThreshold(0);
 _arm2.follow(_arm);
  }

 /*Button Control */
 public void arm_on(double speed)
 {
   _arm.set(ControlMode.PercentOutput, Global_Variables.robot_direction*speed);
  
 }

 public void arm_resetEncoder()
 {
   _arm.setSelectedSensorPosition(0);
 }

 public void arm_angle(double angle)
 {
   _arm.set(ControlMode.MotionMagic, angle, DemandType.AuxPID, 12); //(-59.51556117)*Constants.ARM_CONVERSION)
 }


 public void Arm_Goto_Angle(double angle)
 {
  _arm.set(ControlMode.MotionMagic, angle * Constants.ARM_CONVERSION);  

 }

 public void arm_Middle()
 {
   _arm.set(ControlMode.Position, (-47.82712458)*Constants.ARM_CONVERSION); //  angle*2048*100(gear ratio)/360
 }

 public void arm_Pickup()
 {
   _arm.set(ControlMode.Position, 0);
 }

 public void arm_Default()
 {
   _arm.set(ControlMode.Position, 0);
 }

 public void arm_off()
  {
    _arm.set(ControlMode.PercentOutput,0.0);  
  }

  public double arm_encoder()
  {
    double arm_encoder = _arm.getSelectedSensorPosition()/Constants.ARM_CONVERSION;
    return arm_encoder;
  }

  public double arm_CANCoder()
  {
    double arm_CANCoder = armCANCoder.getAbsolutePosition();
    return arm_CANCoder;
  }

  public double Arm_Speed() 
  {
    return (_arm.getSelectedSensorVelocity());
  }

  public void updateEncoder()
{
  if(arm_CANCoder()!=0)
  {
    _arm.setSelectedSensorPosition((Constants.ARM_OFFSET-armCANCoder.getAbsolutePosition())*Constants.ARM_CONVERSION);  //  angle*2048*100(gear ratio)/360
  }
}

;
  public int getAllianceColor()
  {
    if(DriverStation.getAlliance()==Alliance.Blue)
    {
      return -1;
    }
    else if(DriverStation.getAlliance()==Alliance.Red)
    {
      return 1;
    }
    else
    {
      return 0;
    } 
  }
  
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Right Bumper", Global_Variables.right_bumper);
    SmartDashboard.putBoolean("Left Bumper", Global_Variables.left_bumper);

  }
}
