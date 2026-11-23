package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.util.Map;
import java.util.TreeMap;

import org.team100.foreign.Lib;

public class LevenbergMarquardtParams extends NonlinearOptimizerParams {
    public enum FF {
        LevenbergMarquardtParams(ADDRESS),
        LevenbergMarquardtParams_delete(null, ADDRESS),
        LevenbergMarquardtParams_SetLegacyDefaults(null, ADDRESS),
        LevenbergMarquardtParams_SetCeresDefaults(null, ADDRESS),
        LevenbergMarquardtParams_getVerbosityLM(JAVA_INT, ADDRESS),
        LevenbergMarquardtParams_setVerbosityLM(null, ADDRESS, JAVA_INT);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public enum VerbosityLM {
        SILENT(0), SUMMARY(1), TERMINATION(2), LAMBDA(3), TRYLAMBDA(4), TRYCONFIG(5), DAMPED(6), TRYDELTA(7);

        private final int val;

        private static final Map<Integer, VerbosityLM> all = new TreeMap<>();
        static {
            for (VerbosityLM v : VerbosityLM.values()) {
                all.put(v.val, v);
            }
        }

        VerbosityLM(int val) {
            this.val = val;
        }

        public static VerbosityLM of(int v) {
            return all.get(v);
        }
    }

    LevenbergMarquardtParams(MemorySegment p) {
        super(p, FF.LevenbergMarquardtParams_delete.h);
    }

    public LevenbergMarquardtParams() {
        this((MemorySegment) FF.LevenbergMarquardtParams.invoke());
    }

    public void setLegacyDefaults() {
        FF.LevenbergMarquardtParams_SetLegacyDefaults.invoke(ptr);
    }

    public void setCeresDefaults() {
        FF.LevenbergMarquardtParams_SetCeresDefaults.invoke(ptr);
    }

    /**
     * The verbosity level for Levenberg-Marquardt (default: SILENT), see also
     * NonlinearOptimizerParams::verbosity
     */
    public VerbosityLM getVerbosityLM() {
        return VerbosityLM.of((int) FF.LevenbergMarquardtParams_getVerbosityLM.invoke(ptr));
    }

    public void setVerbosityLM(VerbosityLM value) {
        FF.LevenbergMarquardtParams_setVerbosityLM.invoke(ptr, value.val);
    }

}
