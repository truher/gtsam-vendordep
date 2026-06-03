package gtsam.noiseModel.mEstimator;

import java.lang.foreign.MemorySegment;

import org.team100.foreign.ForeignObject;

public class Base extends ForeignObject {

    protected Base(MemorySegment pointer) {
        super(pointer, null);
    }

}
