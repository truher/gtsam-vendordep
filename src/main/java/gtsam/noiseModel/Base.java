package gtsam.noiseModel;

import java.lang.foreign.MemorySegment;

import org.team100.foreign.ForeignObject;

public class Base extends ForeignObject {

    public Base(MemorySegment p) {
        super(p, null);
    }

}
