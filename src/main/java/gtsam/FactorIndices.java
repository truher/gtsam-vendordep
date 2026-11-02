package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_INT;
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
        FactorIndices_add(null, ADDRESS, JAVA_LONG),
        FactorIndices_size(JAVA_INT, ADDRESS),
        FactorIndices_at(JAVA_INT, ADDRESS, JAVA_INT);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    protected FactorIndices(MemorySegment pointer) {
        super(pointer, FF.FactorIndices_delete.h);
    }

    public FactorIndices() {
        this((MemorySegment) FF.FactorIndices.invoke());
    }

    void add(Key i) {
        FF.FactorIndices_add.invoke(ptr, i.j);
    }

    int size() {
        return (int) FF.FactorIndices_size.invoke(ptr);
    }

    /**
     * This can be confusing.
     * 
     * The FactorIndices object is a vector underneath, containing indices pointing
     * somewhere. This method retrieves indices by their position in the vector, so
     * you can iterate over it without complicating the FFM part.
     * 
     * @param i the index into the FactorIndices object itself
     * @return the index pointing somewhere else.
     */
    int at(int i) {
        return (int) FF.FactorIndices_at.invoke(ptr, i);
    }
}
