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
        Quaternion_rotate(ADDRESS, ADDRESS, ADDRESS),
        Quaternion_norm(JAVA_DOUBLE, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Quaternion(double w, double x, double y, double z) {
        this((MemorySegment) FF.Quaternion.invoke(w, x, y, z));
    }

    protected Quaternion(MemorySegment p) {
        super(p, FF.Quaternion_delete.h);
    }

    public double w() {
        return (double) FF.Quaternion_w.invoke(ptr);
    }

    public double x() {
        return (double) FF.Quaternion_x.invoke(ptr);
    }

    public double y() {
        return (double) FF.Quaternion_y.invoke(ptr);
    }

    public double z() {
        return (double) FF.Quaternion_z.invoke(ptr);
    }

    public Vector4 coeffs() {
        return new Vector4((MemorySegment) FF.Quaternion_coeffs.invoke(ptr));
    }

    public Point3 rotate(Point3 p) {
        return new Point3((MemorySegment) FF.Quaternion_rotate.invoke(ptr, p.ptr));
    }

    public double norm() {
        return (double) FF.Quaternion_norm.invoke(ptr);
    }
}
