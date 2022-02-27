package frc.robot.vision.pixy;

import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class Cargo extends Block {
  public Cargo(int signature, int x, int y, int width, int height, int angle, int index, int age) {
    super(signature, x, y, width, height, angle, index, age);
  }

  public static Cargo fromBlock(Block block) {
    return new Cargo(
      block.getSignature(),
      block.getX(),
      block.getY(),
      block.getWidth(),
      block.getHeight(),
      block.getAngle(),
      block.getIndex(),
      block.getAge()
    );
  }

  public boolean isRed() {
    return this.getSignature() == 1;
  }

  public boolean isBlue() {
    return this.getSignature() == 2;
  }

  public String getColorAsString() {
    if (this.getSignature() == 1) {
      return "Red";
    }
    else if (this.getSignature() == -1) {
      return "None";
    }
    else {
      return "Blue";
    }
  }

  public Color getColor() {
    if (this.getSignature() == 1)
      return Color.RED;
    if (this.getSignature() == 2)
      return Color.BLUE;
    return Color.NONE;
  }


  public enum Color {
    RED(1),
    BLUE(2),
    NONE(-1);

    public final int sig;

    Color(int sig) {
      this.sig = sig;
    }
  }
}
