/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  
  public TalonSRX climberA = new TalonSRX(RobotMap.climberA);
  public TalonSRX climberB = new TalonSRX(RobotMap.climberB);
  public VictorSPX climberC = new VictorSPX(RobotMap.climberC);

  public Solenoid climbRelease = new Solenoid(RobotMap.climbRelease);

  public Climber() {
    climberC.setSelectedSensorPosition(0);
    climberA.setInverted(true);
    climberB.setInverted(true);
    climberB.follow(climberA);
    climberC.setNeutralMode(NeutralMode.Brake);
  }

  public void actuateClimb() {
    climbRelease.set(true);
  }

  public void deploy(double percentOutput) {
    climberC.set(ControlMode.PercentOutput, Math.abs(percentOutput));
  }

  public void release(double percentOutput) {
    climberC.set(ControlMode.PercentOutput, percentOutput * -1);
  }

  public void retract(double power) {
    climberA.set(ControlMode.PercentOutput, (power * -1));
  }

  public void extend(double power){
    climberA.set(ControlMode.PercentOutput, Math.abs(power));;
  }

  public void stop() {
    climberA.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
  }
}
