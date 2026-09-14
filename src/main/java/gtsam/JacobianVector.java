package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * a typedef:
 * see gtsam/nonlinear/CustomFactor.h
 * using JacobianVector = std::vector<Matrix>;
 * 
 * This does not own the pointer; never deleted.
 */
public class JacobianVector extends ForeignObject {
    public enum FF {
        JacobianVector_insert(null, ADDRESS, JAVA_INT, ADDRESS),
        JacobianVector_insertMatrix3(null, ADDRESS, JAVA_INT, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /** Does not own the pointer */
    private JacobianVector(MemorySegment p) {
        super(p, null);
    }

    public static JacobianVector fromPointer(MemorySegment p) {
        if (p.equals(MemorySegment.NULL))
            return null;
        return new JacobianVector(p);
    }

    public void insert(int i, Matrix m) throws Throwable {
        FF.JacobianVector_insert.h.invokeExact(ptr, i, m.ptr);
    }

    public void insert(int i, Matrix3 m) throws Throwable {
        FF.JacobianVector_insertMatrix3.h.invokeExact(ptr, i, m.ptr);
    }
}
