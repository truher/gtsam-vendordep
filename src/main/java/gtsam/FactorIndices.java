package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class FactorIndices extends ForeignObject {

    public enum FF {
        FactorIndices(ADDRESS),
        FactorIndices_delete(null, ADDRESS),
        FactorIndices_add(null, ADDRESS, JAVA_LONG);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    protected FactorIndices(MemorySegment pointer) {
        super(pointer, FF.FactorIndices_delete.h);
    }

    public FactorIndices() throws Throwable {
        this((MemorySegment) FF.FactorIndices.h.invokeExact());
    }

    void add(Key i) throws Throwable {
        FF.FactorIndices_add.h.invokeExact(ptr, i.j);
    }
}
