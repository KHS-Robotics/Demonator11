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
public class RobotMap {
  public static final int XBOX_PORT = 0;
  public static final int SWITCHBOX_PORT = 1;

  public static final int FRONT_LEFT_PIVOT = 6;
  public static final int FRONT_RIGHT_PIVOT = 5;
  public static final int REAR_LEFT_PIVOT = 7;
  public static final int REAR_RIGHT_PIVOT = 4;

  public static final int FRONT_LEFT_DRIVE = 2;
  public static final int FRONT_RIGHT_DRIVE = 1;
  public static final int REAR_LEFT_DRIVE = 3;
  public static final int REAR_RIGHT_DRIVE = 20;

  public static final int FRONT_LEFT_DIGITAL_INPUT = 2;
  public static final int FRONT_RIGHT_DIGITAL_INPUT = 1;
  public static final int REAR_LEFT_DIGITAL_INPUT = 3;
  public static final int REAR_RIGHT_DIGITAL_INPUT = 0;

  public static final int INTAKE_DRIVE = 9;
  public static final int INTAKE_POSITION = 8;

  public static final int INDEXER_FLOOR = 10;
  public static final int INDEXER_SIDE = 18;
  public static final int INDEXER_FEEDER = 11;

  public static final int SHOOTER_LEADER = 13;
  public static final int SHOOTER_FOLLOWER = 12;

  public static final int HOOD_SERVO_1 = 0;
  public static final int HOOD_SERVO_2 = 1;

  public static final int ELEVATOR_LEADER = 14;
  public static final int ELEVATOR_FOLLOWER1 = 15;
  public static final int ELEVATOR_FOLLOWER2 = 16;
  public static final int PIVOT_MOTOR = 17;
}