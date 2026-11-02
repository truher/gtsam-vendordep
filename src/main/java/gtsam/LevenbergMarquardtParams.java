package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class LevenbergMarquardtParams extends ForeignObject {
    public enum FF {
        LevenbergMarquardtParams(ADDRESS),
        LevenbergMarquardtParams_delete(null, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    LevenbergMarquardtParams(MemorySegment p) {
        super(p, FF.LevenbergMarquardtParams_delete.h);
    }

    public LevenbergMarquardtParams() {
        this((MemorySegment) FF.LevenbergMarquardtParams.invoke());
    }

}
