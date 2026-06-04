package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/** this is really an Eigen type. */
public class Quaternion extends ForeignObject {
    public enum FF {
        Quaternion(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Quaternion_delete(null, ADDRESS),
        Quaternion_w(JAVA_DOUBLE, ADDRESS),
        Quaternion_x(JAVA_DOUBLE, ADDRESS),
        Quaternion_y(JAVA_DOUBLE, ADDRESS),
        Quaternion_z(JAVA_DOUBLE, ADDRESS),
        Quaternion_coeffs(ADDRESS, ADDRESS),
        Quaternion_rotate(ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Quaternion(double w, double x, double y, double z) throws Throwable {
        this((MemorySegment) FF.Quaternion.h.invokeExact(w, x, y, z));
    }

    protected Quaternion(MemorySegment p) {
        super(p, FF.Quaternion_delete.h);
    }

    public double w() throws Throwable {
        return (double) FF.Quaternion_w.h.invokeExact(ptr);
    }

    public double x() throws Throwable {
        return (double) FF.Quaternion_x.h.invokeExact(ptr);
    }

    public double y() throws Throwable {
        return (double) FF.Quaternion_y.h.invokeExact(ptr);
    }

    public double z() throws Throwable {
        return (double) FF.Quaternion_z.h.invokeExact(ptr);
    }

    public Vector4 coeffs() throws Throwable {
        return new Vector4((MemorySegment) FF.Quaternion_coeffs.h.invokeExact(ptr));
    }

    public Point3 rotate(Point3 p) throws Throwable {
        return new Point3((MemorySegment) FF.Quaternion_rotate.h.invokeExact(ptr, p.ptr));
    }
}
