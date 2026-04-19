package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * a typedef:
 * see gtsam/nonlinear/CustomFactor.h
 * using JacobianVector = std::vector<Matrix>;
 * 
 * This does not own the pointer.
 */
public class JacobianVector {
    // this is never deleted, it is owned by the caller
    // private static final MethodHandle JacobianVector_delete = Lib.downVoid(
    // "JacobianVector_delete", ADDRESS);
    private static final MethodHandle JacobianVector_insert = Lib.downVoid(
            "JacobianVector_insert", ADDRESS, JAVA_INT, ADDRESS);
    private static final MethodHandle JacobianVector_insertMatrix3 = Lib.downVoid(
            "JacobianVector_insertMatrix3", ADDRESS, JAVA_INT, ADDRESS);

    public final MemorySegment ptr;

    /** Does not own the pointer */
    private JacobianVector(MemorySegment p) {
        ptr = p;
    }

    public static JacobianVector fromPointer(MemorySegment p) {
        if (p.equals(MemorySegment.NULL))
            return null;
        return new JacobianVector(p);
    }

    public void insert(int i, Matrix m) throws Throwable {
        JacobianVector_insert.invokeExact(ptr, i, m.ptr);
    }

    public void insert(int i, Matrix3 m) throws Throwable {
        JacobianVector_insertMatrix3.invokeExact(ptr, i, m.ptr);
    }
}
