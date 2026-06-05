package gtsam;

import java.lang.foreign.MemorySegment;

public class NoiseModelFactor extends NonlinearFactor {

    NoiseModelFactor(MemorySegment p) {
        super(p);
    }
    
}
