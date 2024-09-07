// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WestCoastModule extends SubsystemBase {
  /** Creates a new westCoastModule. */
  private final CANSparkMax forwardModule;
  private final CANSparkMax backModule;

  private final RelativeEncoder forwardEncoder;
  private final RelativeEncoder backEncoder;

  private final CANcoder absolutedEncoder;

  private final CANcoderConfiguration absolutedEncoderConfig;

  public WestCoastModule(int forwardModule_ID, int backModule_ID, int absolutedEncoder_ID, double absolutedEncoderOffset, boolean forwardModuleReserve, boolean backModuleReserve) {
    forwardModule = new CANSparkMax(forwardModule_ID, MotorType.kBrushless);
    backModule = new CANSparkMax(backModule_ID, MotorType.kBrushless);

    forwardEncoder = forwardModule.getEncoder();
    backEncoder = backModule.getEncoder();

    forwardModule.setInverted(forwardModuleReserve);
    backModule.setInverted(backModuleReserve);

    forwardModule.setIdleMode(IdleMode.kBrake);
    backModule.setIdleMode(IdleMode.kBrake);

    backModule.follow(forwardModule);

    absolutedEncoder = new CANcoder(absolutedEncoder_ID);

    absolutedEncoderConfig = new CANcoderConfiguration();

    absolutedEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    absolutedEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absolutedEncoderConfig.MagnetSensor.MagnetOffset = absolutedEncoderOffset;

    absolutedEncoder.getConfigurator().apply(absolutedEncoderConfig);
  }

  public CANSparkMax getForwardModule() {
    return forwardModule;
  }

  public double getForwardPosition() {
    return forwardEncoder.getPosition();
  }

  public double getBackPosition() {
    return backEncoder.getPosition();
  }


  public double getForwardVelocity() {
    return forwardEncoder.getVelocity();
  }

  public double getBackVelocity() {
    return backEncoder.getVelocity();
  }

  public void setModule(double speed) {
    forwardModule.set(speed);
  }

  public void setDesiredState_Auto(DifferentialDriveWheelSpeeds wheelSpeeds) {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}