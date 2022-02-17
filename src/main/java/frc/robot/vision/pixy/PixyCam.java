package frc.robot.vision.pixy;

import java.util.ArrayList;

import javax.imageio.plugins.tiff.GeoTIFFTagSet;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class PixyCam {
    private final Pixy2 pixy;
    private int blockCount;
    private Cargo[] cargos = {};

    public PixyCam() {
        pixy = Pixy2.createInstance(new SPILink());
		pixy.init();
        
        var tab = Shuffleboard.getTab("Pixy");
        tab.addNumber("BlockCount", () -> blockCount);
        tab.addBoolean("Red", this::hasRedInFrame);
        tab.addBoolean("Blue", this::hasBlueInFrame);
    }

    public void setLamp(boolean on) {
        if (on)
            pixy.setLamp((byte) 1, (byte) 1);
        else
            pixy.setLamp((byte) 0, (byte) 0);
    }

    public void updateCargoInFrame() {
        blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1 | Pixy2CCC.CCC_SIG2, 2);
        if (blockCount >= 0) {
            ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
            cargos = new Cargo[blocks.size()];
            for (int i = 0; i < cargos.length; i++) {
                cargos[i] = Cargo.fromBlock(blocks.get(i));
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

    public int nextCargo() {
        if(cargos.length == 0) {
            return 0;
        }
        if(cargos.length == 1) {
            return cargos[0].getSignature();
        }
        if(cargos[1].getY() > cargos[0].getY()) {
            return cargos[0].getSignature();
        } else {
            return cargos[1].getSignature();
        }
    }

    public boolean isMerged() {
        //847098: frame area
        return cargos.length == 1 && cargos[0].getWidth() * cargos[0].getHeight() > 847098 * 0.5;
    }

    public int getNumCargo() {
        return cargos.length + (isMerged() ? 1 : 0);
    }
}
