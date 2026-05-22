package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Vector3 extends ForeignObject implements Manifold<Vector3> {
    private static final MethodHandle Vector3 = Lib.down(
            "Vector3", ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Vector3_delete = Lib.downVoid(
            "Vector3_delete", ADDRESS);
    private static final MethodHandle Vector3_at = Lib.down(
            "Vector3_at", JAVA_DOUBLE, ADDRESS, JAVA_INT);
    private static final MethodHandle Vector3_print = Lib.downVoid(
            "Vector3_print", ADDRESS);
    private static final MethodHandle Vector3_minus = Lib.down(
            "Vector3_minus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector3_plus = Lib.down(
            "Vector3_plus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector3_equals = Lib.down(
            "Vector3_equals", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);

    public static class Traits implements Manifold.Traits<Vector3> {

    }

    public static final Traits traits = new Traits();

        @Override
    public Traits traits() {
        return traits;
    }

    public Vector3(MemorySegment p) {
        super(p, Vector3_delete);
    }

    public Vector3(double v0, double v1, double v2) throws Throwable {
        this((MemorySegment) Vector3.invokeExact(v0, v1, v2));
    }

    public double at(int i) throws Throwable {
        return (double) Vector3_at.invokeExact(ptr, i);
    }

    public void print() throws Throwable {
        Vector3_print.invokeExact(ptr);
    }

    @Override
    public String toString() {
        try {
            StringBuilder b = new StringBuilder();
            int rows = 3;
            b.append("[");
            for (int r = 0; r < rows; ++r) {
                b.append(String.format(" %8.5f ", at(r)));
            }
            b.append("]\n");
            return b.toString();
        } catch (Throwable e) {
            e.printStackTrace();
            return "";
        }
    }

    @Override
    public int dimension() throws Throwable {
        return 3;
    }

    public Vector3 minus(Vector3 other) throws Throwable {
        return new Vector3((MemorySegment) Vector3_minus.invokeExact(ptr, other.ptr));
    }

    public Vector3 plus(Vector3 other) throws Throwable {
        return new Vector3((MemorySegment) Vector3_plus.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector local(Vector3 other) throws Throwable {
        return new Vector(other.minus(this));
    }

    @Override
    public Vector3 retract(Vector v) throws Throwable {
        // TODO: better conversion.
        Vector3 v3 = new Vector3(v.at(0), v.at(1), v.at(2));
        return plus(v3);
    }

    public boolean equals(Vector3 other, double tol) throws Throwable {
        return (boolean) Vector3_equals.invokeExact(ptr, other.ptr, tol);
    }

}
