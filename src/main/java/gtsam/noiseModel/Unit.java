package gtsam.noiseModel;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.Matrix;
import gtsam.Vector;
import gtsam.shared_ptr;

/**
 * shared_ptr manages the lifecycle of Unit: parent ForeignObject deleter is
 * null.
 */
public class Unit extends Isotropic {
    public enum FF {
        noiseModel_Unit_Create(ADDRESS, JAVA_INT),
        noiseModel_Unit_CreateVector(ADDRESS, ADDRESS),
        noiseModel_Unit_CreateMatrix(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Unit(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Unit> Create(int dim) throws Throwable {
        return new shared_ptr<Unit>((MemorySegment) FF.noiseModel_Unit_Create.h.invokeExact(dim),
                Unit::new);
    }

    public static shared_ptr<Unit> Create(Vector v) throws Throwable {
        return new shared_ptr<Unit>((MemorySegment) FF.noiseModel_Unit_CreateVector.h.invokeExact(v.ptr),
                Unit::new);
    }

    public static shared_ptr<Unit> Create(Matrix m) throws Throwable {
        return new shared_ptr<Unit>((MemorySegment) FF.noiseModel_Unit_CreateMatrix.h.invokeExact(m.ptr),
                Unit::new);
    }
}
