package gtsam;

import java.lang.foreign.MemorySegment;

import org.team100.foreign.ForeignObject;

public class ISAM2 extends ForeignObject {

    ISAM2(MemorySegment pointer) {
        // observed only
        super(pointer, null);
    }

}
