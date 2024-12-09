// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final TalonFX shooterMotor;
  public ShooterSubsystem() {
    shooterMotor = new TalonFX(ShooterConstants.ShooterMotor_ID);

    shooterMotor.setInverted(ShooterConstants.shooterReserve);

    shooterMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getShooterSpeed() {
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  public void shootAMP() {
    shooterMotor.setVoltage(ShooterConstants.shootAMPVol);
  }

  public void shootSpeaker() {
    shooterMotor.setVoltage(ShooterConstants.shootSpeakerVol);
  }

  public void passNote() {
    shooterMotor.setVoltage(ShooterConstants.passNoteVol);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter/Velocity", getShooterSpeed());
  }
}
