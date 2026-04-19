package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Point3 extends ForeignObject {
    private static final MethodHandle Point3 = Lib.down(
            "Point3", ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Point3_delete = Lib.downVoid(
            "Point3_delete", ADDRESS);
    private static final MethodHandle Point3_print = Lib.downVoid(
            "Point3_print", ADDRESS);

    public Point3(MemorySegment p) {
        super(p, Point3_delete);
    }

    public Point3(double x, double y, double z) throws Throwable {
        this((MemorySegment) Point3.invokeExact(x, y, z));
    }

    public void print() throws Throwable {
        Point3_print.invokeExact(ptr);
    }

}
