// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final TalonFX intakeWheel;

  private final 
  public IntakeSubsystem() {
    intakeWheel = new TalonFX(IntakeConstants.intakeWheel_ID);

    intakeWheel.setNeutralMode(null);

    intakeWheel.setInverted(false);

  }

  public void intake() {
    intakeWheel.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
