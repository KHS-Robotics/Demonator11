package frc.robot.vision.pixy;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class PixyCam {
    private final Pixy2 pixy;
    private int blockCount;

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

    public Cargo[] getCargoInFrame() {
        blockCount = pixy.getCCC().getBlocks(true, Pixy2CCC.CCC_SIG_ALL, 2);
		if (blockCount <= 0) {
			return new Cargo[]{};
		}
        ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
        Cargo[] cargos = new Cargo[blocks.size()];
        for (int i = 0; i < cargos.length; i++) {
            cargos[i] = Cargo.fromBlock(blocks.get(i));
        }
        return cargos;
    }

    public boolean hasRedInFrame() {
        Cargo[] cargos = getCargoInFrame();
        if (cargos.length == 1)
            return cargos[0].isRed();
        if (cargos.length == 2)
            return cargos[0].isRed() || cargos[1].isRed();
        return false;
    }

    public boolean hasBlueInFrame() {
        Cargo[] cargos = getCargoInFrame();
        if (cargos.length == 1)
            return cargos[0].isBlue();
        if (cargos.length == 2)
            return cargos[0].isBlue() || cargos[1].isBlue();
        return false;
    }
}
