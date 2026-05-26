package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * TODO: Point3 is a typedef of Vector3, so this seems wrong.
 */
public class Point3 extends ForeignObject {
    public enum FF {
        Point3(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Point3_delete(null, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Point3(MemorySegment p) {
        super(p, FF.Point3_delete.h);
    }

    public Point3(double x, double y, double z) throws Throwable {
        this((MemorySegment) FF.Point3.h.invokeExact(x, y, z));
    }

}
