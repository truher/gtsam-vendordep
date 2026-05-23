package gtsam;

// TODO: include dimension as a type parameter.
// TODO: add method handle maker here?
public interface VectorType<V extends VectorType<V>> {
    int dimension() throws Throwable;

    double at(int i) throws Throwable;

    void set(int i, double val) throws Throwable;

    V plus(V other) throws Throwable;

    V minus(V other) throws Throwable;

    V times(double a) throws Throwable;
}
