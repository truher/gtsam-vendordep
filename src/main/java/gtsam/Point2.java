package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Point2 extends ForeignObject {
    private static final MethodHandle Point2 = Lib.down(
            "Point2", ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Point2_delete = Lib.downVoid(
            "Point2_delete", ADDRESS);
    private static final MethodHandle Point2_x = Lib.down(
            "Point2_x", JAVA_DOUBLE, ADDRESS);
    private static final MethodHandle Point2_y = Lib.down(
            "Point2_y", JAVA_DOUBLE, ADDRESS);
    private static final MethodHandle Point2_print = Lib.downVoid(
            "Point2_print", ADDRESS);

    public Point2(MemorySegment p) {
        super(p, Point2_delete);
    }

    public Point2(double x, double y) throws Throwable {
        this((MemorySegment) Point2.invokeExact(x, y));
    }

    public double x() throws Throwable {
        return (double) Point2_x.invokeExact(ptr);
    }

    public double y() throws Throwable {
        return (double) Point2_y.invokeExact(ptr);
    }

    public void print() throws Throwable {
        Point2_print.invokeExact(ptr);
    }

}
