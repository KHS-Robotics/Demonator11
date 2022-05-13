package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.io.ByteArrayOutputStream;
import java.io.ObjectOutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class LEDController extends SubsystemBase {
  private int counter = 0;
  private State state;
  private DatagramSocket socket;

  public LEDController() {
    try {
      byte[] piIP = new byte[]{(byte) 10, (byte) 43, (byte) 42, (byte) 13};
      socket = new DatagramSocket();
      socket.connect(InetAddress.getByAddress(piIP), 8123);
    } catch (Exception e) {
      // might not want it to print here or in sendPacket()
      e.printStackTrace();
    }
  }


  public void changeDisabled() {
    if (Robot.color.equals(DriverStation.Alliance.Blue)) {
      state = State.DISABLEDBLUE;
    } else {
      state = State.DISABLEDRED;
    }
  }

  public void changeAuto() {
    if (Robot.color.equals(DriverStation.Alliance.Blue)) {
      state = State.AUTOBLUE;
    } else {
      state = State.AUTORED;
    }
  }

  public void changeTeleRed() {
    if (RobotContainer.pixy.hasBlueInFrame()) {
      state = State.TELEWRONG;
      return;
    }
    switch (RobotContainer.pixy.getNumCargo()) {
      case 0: state = State.TELEZERO; break;
      case 1: state = State.TELEONE; break;
      case 2: state = State.TELETWO;
    }
  }

  public void changeTeleBlue() {
    if (RobotContainer.pixy.hasRedInFrame()) {
      state = State.TELEWRONG;
      return;
    }
    switch (RobotContainer.pixy.getNumCargo()) {
      case 0: state = State.TELEZERO; break;
      case 1: state = State.TELEONE; break;
      case 2: state = State.TELETWO;
    }
  }

  public void sendPacket() {
    try {
      ByteArrayOutputStream byteOutput = new ByteArrayOutputStream(1024);
      ObjectOutputStream objectOutput = new ObjectOutputStream(byteOutput);
      objectOutput.writeObject(state);
      socket.send(new DatagramPacket(byteOutput.toByteArray(), 1024));
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    // one send a packet every 10 ticks
    if (counter % 10 == 0) {
      sendPacket();
      counter = 0;
    }
    counter++;
  }
}

enum State {
  DISABLEDRED,
  DISABLEDBLUE,
  AUTORED,
  AUTOBLUE,
  TELEZERO,
  TELEONE,
  TELETWO,
  TELEWRONG
}
