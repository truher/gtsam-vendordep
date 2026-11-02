package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Vector2 extends ForeignObject
        implements VectorType<Vector2>, Manifold<Vector2, Vector2> {
    public enum FF {
        Vector2(ADDRESS),
        Vector2_delete(null, ADDRESS),
        Vector2_at(JAVA_DOUBLE, ADDRESS, JAVA_INT),
        Vector2_set(null, ADDRESS, JAVA_INT, JAVA_DOUBLE),
        Vector2_plus(ADDRESS, ADDRESS, ADDRESS),
        Vector2_minus(ADDRESS, ADDRESS, ADDRESS),
        Vector2_times(ADDRESS, ADDRESS, JAVA_DOUBLE);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    @Override
    public Vector2 dxZero() {
        return new Vector2(0, 0);
    }

    public Vector2(MemorySegment p) {
        super(p, FF.Vector2_delete.h);
    }

    public Vector2() {
        this((MemorySegment) FF.Vector2.invoke());
    }

    public Vector2(Vector v) {
        this(v.at(0), v.at(1));
    }

    public Vector2(double v0, double v1) {
        this();
        set(0, v0);
        set(1, v1);
    }

    @Override
    public int dimension() {
        return 2;
    }

    public double at(int i) {
        return (double) FF.Vector2_at.invoke(ptr, i);
    }

    @Override
    public void set(int i, double val) {
        FF.Vector2_set.invoke(ptr, i, val);
    }

    @Override
    public Vector2 plus(Vector2 other) {
        return new Vector2((MemorySegment) FF.Vector2_plus.invoke(ptr, other.ptr));
    }

    @Override
    public Vector2 minus(Vector2 other) {
        return new Vector2((MemorySegment) FF.Vector2_minus.invoke(ptr, other.ptr));
    }

    @Override
    public Vector2 times(double a) {
        return new Vector2((MemorySegment) FF.Vector2_times.invoke(ptr, a));
    }

    @Override
    public Vector2 local(Vector2 other) {
        return other.minus(this);
    }

    @Override
    public Vector2 retract(Vector2 v) {
        return new Vector2((MemorySegment) FF.Vector2_plus.invoke(ptr, v.ptr));
    }

    @Override
    public Vector2 local(Vector2 g, Matrix H1, Matrix H2) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'localCoordinates'");
    }

    @Override
    public Vector2 retract(Vector2 v, Matrix H1, Matrix H2) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'retract'");
    }

}
