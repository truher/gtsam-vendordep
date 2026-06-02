package gtsam.noiseModel;

import java.lang.foreign.MemorySegment;

public class Isotropic extends Diagonal {

    public Isotropic(MemorySegment p) {
        super(p);
    }
    
}
