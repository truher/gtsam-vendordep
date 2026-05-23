package gtsam;

import java.lang.foreign.MemorySegment;

import org.team100.foreign.ForeignObject;

// TODO: finish implementation
public class Vector6 extends ForeignObject implements VectorType<Vector6> {
    public Vector6(MemorySegment p) {
        super(null, null);
    }

    public Vector6(//
            double v0, double v1, double v2, //
            double v3, double v4, double v5) throws Throwable {
        this(null);
    }

    @Override
    public int dimension() throws Throwable {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'dimension'");
    }

    @Override
    public double at(int i) throws Throwable {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'at'");
    }

    @Override
    public void set(int i, double val) throws Throwable {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'set'");
    }

    @Override
    public Vector6 plus(Vector6 other) throws Throwable {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'plus'");
    }

    @Override
    public Vector6 minus(Vector6 other) throws Throwable {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'minus'");
    }

    @Override
    public Vector6 times(double a) throws Throwable {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'times'");
    }
}
