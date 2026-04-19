package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class LevenbergMarquardtParams extends ForeignObject {
    private static final MethodHandle LevenbergMarquardtParams = Lib.down(
            "LevenbergMarquardtParams", ADDRESS);
    private static final MethodHandle LevenbergMarquardtParams_delete = Lib.downVoid(
            "LevenbergMarquardtParams_delete", ADDRESS);

    public LevenbergMarquardtParams() throws Throwable {
        this((MemorySegment) LevenbergMarquardtParams.invokeExact());
    }

    public LevenbergMarquardtParams(MemorySegment p) {
        super(p, LevenbergMarquardtParams_delete);
    }

}
