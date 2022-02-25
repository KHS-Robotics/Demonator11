/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class Constants {
  public static final double SENS = 0.5;

  public static final double ROBOT_HEIGHT = 0.6;
  public static final double TARGET_HEIGHT = 2.64;

  public static final double FRONT_LEFT_P = 0.015;
  public static final double FRONT_LEFT_I = 0.0;
  public static final double FRONT_LEFT_D = 0.0;
  
  public static final double FRONT_RIGHT_P = 0.015;
  public static final double FRONT_RIGHT_D = 0.0;
  public static final double FRONT_RIGHT_I = 0.0;

  public static final double REAR_LEFT_P = 0.015;
  public static final double REAR_LEFT_I = 0.0;
  public static final double REAR_LEFT_D = 0.0;

  public static final double REAR_RIGHT_P = 0.015;
  public static final double REAR_RIGHT_I = 0.0;
  public static final double REAR_RIGHT_D = 0.0;

  public static final double DRIVE_VEL_ENCODER = 0.000637;
  public static final double DRIVE_POS_ENCODER = 0.038318;

  public static final double FRONT_RIGHT_DRIVE_P = 0.15;
  public static final double FRONT_RIGHT_DRIVE_I = 0.00055;
  public static final double FRONT_RIGHT_DRIVE_D = 3.0;
  public static final double FRONT_RIGHT_DRIVE_FF = 0.2857;

  public static final double FRONT_LEFT_DRIVE_P = 0.15;
  public static final double FRONT_LEFT_DRIVE_I = 0.00055;
  public static final double FRONT_LEFT_DRIVE_D = 3.0;
  public static final double FRONT_LEFT_DRIVE_FF = 0.2857;

  public static final double REAR_RIGHT_DRIVE_P = 0.15;
  public static final double REAR_RIGHT_DRIVE_I = 0.00055;
  public static final double REAR_RIGHT_DRIVE_D = 3.0;
  public static final double REAR_RIGHT_DRIVE_FF = 0.2857;

  public static final double REAR_LEFT_DRIVE_P = 0.15;
  public static final double REAR_LEFT_DRIVE_I = 0.00055;
  public static final double REAR_LEFT_DRIVE_D = 3.0;
  public static final double REAR_LEFT_DRIVE_FF = 0.2857;

  public static final double TARGET_P = 0.0275;
  public static final double TARGET_I = 0.001;
  public static final double TARGET_D = 0.0001;

  public static final double SHOOTER_P = 0.0006;
  public static final double SHOOTER_I = 0.000001;
  public static final double SHOOTER_D = 0.02;
  public static final double SHOOTER_FF = 0.00017024;
}