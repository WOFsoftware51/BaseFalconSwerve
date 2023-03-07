// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase 
{  

  private final TalonFX _intake = new TalonFX(Constants.Intake_.Intake_Motor);
  public int variable;
  
  /** Creates a new Intake. */
  public void intake_init() 
  {
    _intake.setNeutralMode(NeutralMode.Brake);
    _intake.setInverted(false);

 /*_.config_kF(0, 0 , 30);
 _arm.config_kP(0, 0, 30);
 _arm.config_kI(0, 0, 30);
 _arm.config_kD(0, 0, 30);
 _arm.setSensorPhase(true);*/
  }

  
  /*Button Control */
   
        /*Joystick Control*/
  public void Intake_On() 
  {
    _intake.set(ControlMode.PercentOutput,0.60);
  }
 
  public void Intake_Slow()
  {
    _intake.set(ControlMode.PercentOutput,0.075);  
  }

  public void Intake_Off()
  {
    _intake.set(ControlMode.PercentOutput,0.0);  
  }

  public void Intake_Reverse()
  {
    _intake.set(ControlMode.PercentOutput,-0.25);  
  }

  public double Intake_Speed() 
  {
    return (_intake.getSelectedSensorVelocity()/2048.0);
  }

  public double Intake_Current() 
  {
    return _intake.getStatorCurrent();
  }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
