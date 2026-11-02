package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.SegmentAllocator;
import java.lang.foreign.StructLayout;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;
import org.team100.foreign.Pairs;

public class GaussianFactor extends Factor {
    public enum FF {
        GaussianFactor_jacobian(Pairs.PtrPair, ADDRESS);

        private final MethodHandle h;

        private FF(StructLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    GaussianFactor(MemorySegment p) {
        super(p);
    }

    public Pair<Matrix, Vector> jacobian() {
        MemorySegment resultStruct = (MemorySegment) FF.GaussianFactor_jacobian.invoke(
                (SegmentAllocator) Lib.arena, ptr);
        MemorySegment firstPtr = (MemorySegment) Pairs.PtrPair_first.get(resultStruct, 0);
        MemorySegment secondPtr = (MemorySegment) Pairs.PtrPair_second.get(resultStruct, 0);
        return new Pair<>(new Matrix(firstPtr), new Vector(secondPtr));
    }

}
