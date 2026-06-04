package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.SegmentAllocator;
import java.lang.foreign.StructLayout;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;
import org.team100.foreign.Pairs;

public class GaussianFactor {
    public enum FF {
        GaussianFactor_jacobian(Pairs.PtrPair, ADDRESS);

        public final MethodHandle h;

        FF(StructLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /**
     * Pointer to a shared pointer that points at the factor, because that's how
     * these are produced, e.g. by NonlinearFactor.linearize().
     */
    final MemorySegment sharedPtrPtr;

    GaussianFactor(MemorySegment p) {
        sharedPtrPtr = p;
    }

    public Pair<Matrix, Vector> jacobian() throws Throwable {
        MemorySegment resultStruct = (MemorySegment) FF.GaussianFactor_jacobian.h.invokeExact(
                (SegmentAllocator) Lib.arena, sharedPtrPtr);
        MemorySegment firstPtr = (MemorySegment) Pairs.PtrPair_first.get(resultStruct, 0);
        MemorySegment secondPtr = (MemorySegment) Pairs.PtrPair_second.get(resultStruct, 0);
        return new Pair<>(new Matrix(firstPtr), new Vector(secondPtr));
    }

}
