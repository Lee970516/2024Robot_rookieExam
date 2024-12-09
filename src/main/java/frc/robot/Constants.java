// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverController_ID = 0;
    public static final int operatorController_ID = 1;
  }

  public static class ShooterConstants {
    public static final int ShooterMotor_ID = 0;
    public static final boolean shooterReserve = false;

    public static final double shootAMPVol = 0;
    public static final double shootSpeakerVol = 0;
    public static final double passNoteVol = 0;

  }

  public static class IntakeConstants {
    public static final int intakeWheel_ID = 0;
    public static final boolean intakeWheelReserve = false;

    public static final double intakeVol = 0;
    public static final double outNoteVol = 0;

  }

  public static class WestCoastConstants {
    public static final int rightForwardModule_ID = 0;
    public static final int rightBackModule_ID = 0;
    public static final int leftForwardModule_ID = 0;
    public static final int leftBackModule_ID = 0;

    public static final int rightAbsolutedEncoder_ID = 0;
    public static final int leftAbsolutedEncoder_ID = 0;

    public static final double rightOffSet = 0;
    public static final double leftOffSet = 0;

    public static final boolean rightForwardModuleReserve = true;
    public static final boolean rightBackModuleReserve = true;
    public static final boolean leftForwardModuleReserve = true;
    public static final boolean leftBackModuleReserve = true;

    public static final double maxDirveMotorSpeed = 0;


    public static final DifferentialDriveKinematics westCoastKinematic = new DifferentialDriveKinematics(0);//軌道寬度

    public static final int gyro_ID = 0;


  }
}
