package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.pixy.Cargo;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class PixyCam extends SubsystemBase {
  private final Pixy2 pixy;
  private int blockCount;
  private Cargo[] cargos = new Cargo[2];
  private ArrayList<Block> blocks;

  public PixyCam() {
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init();
    setLamp(true);

    var tab = Shuffleboard.getTab("Pixy");
    tab.addNumber("BlockCount", () -> blockCount);
    tab.addNumber("CargoCount", () -> getNumCargo());
    tab.addBoolean("Red", this::hasRedInFrame);
    tab.addBoolean("Blue", this::hasBlueInFrame);
    tab.addString("NextColor", () -> this.nextCargo().getColor().toString());
  }

  @Override
  public void periodic() {
  }

  public void setLamp(boolean on) {
    if (on)
      pixy.setLamp((byte) 1, (byte) 0);
    else
      pixy.setLamp((byte) 0, (byte) 0);
  }

  public void updateCargoInFrame() {
    blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1 | Pixy2CCC.CCC_SIG2, 2);
    if (blockCount >= 0) {
      blocks.clear();

      blocks = pixy.getCCC().getBlockCache();
      //cargos = new Cargo[blocks.size()];
      for (int i = 0; i < cargos.length; i++) {
        if(blocks.size() > i ) {
          cargos[i] = Cargo.fromBlock(blocks.get(i));
        } else {
          cargos[i] = Cargo.fromBlock(new Block(-1, 0, 0, 0, 0, 0, 0, 0));
        }
      }
    }
  }

  public boolean hasRedInFrame() {
    if (cargos.length == 1)
      return cargos[0].isRed();
    if (cargos.length == 2)
      return cargos[0].isRed() || cargos[1].isRed();
    return false;
  }

  public boolean hasBlueInFrame() {
    if (cargos.length == 1)
      return cargos[0].isBlue();
    if (cargos.length == 2)
      return cargos[0].isBlue() || cargos[1].isBlue();
    return false;
  }

  public boolean hasBothInFrame() {
    return (hasBlueInFrame() && hasRedInFrame());
  }

  public int nextCargoSignature() {
    if (cargos.length == 0) {
      return 0;
    } else if (cargos.length == 1) {
      return cargos[0].getSignature();
    } else {
      if (cargos[1].getY() > cargos[0].getY() && cargos[1].getSignature() > 0) {
        return cargos[1].getSignature();
      } else {
        return cargos[0].getSignature();
      }
    }
  }

  public Cargo nextCargo() {
    if (cargos.length == 0) {
      return new Cargo(Cargo.Color.NONE.sig, 0, 0, 0, 0, 0, 0, 0);
    } else if (cargos.length == 1) {
      return cargos[0];
    } else {
      if (cargos[1].getY() > cargos[0].getY() && cargos[1].getSignature() > 0) {
        return cargos[1];
      } else {
        return cargos[0];
      }
    }
  }

  public boolean isMerged() {
    //847098: frame area
    return cargos.length == 1 && cargos[0].getWidth() * cargos[0].getHeight() > 60000;
  }

  public int getNumCargo() {
    return cargos.length + (isMerged() ? 1 : 0);
  }

  /**
   * 0 = left, 1 = right
   * 
   * @return
   */
  public boolean nextCargoLeft() {
    int frontIndex = 0;
    if (cargos.length > 1 && cargos[1].getY() > cargos[0].getY()) {
      frontIndex = 1;
    }
    
    return cargos[frontIndex].getX() < pixy.getFrameWidth() / 2;
  }
}
