package gtsam;

// TODO: include dimension as a type parameter.
// TODO: add method handle maker here?
public interface VectorType<V extends VectorType<V>> {
    int dimension();

    double at(int i);

    void set(int i, double val);

    V plus(V other);

    V minus(V other);

    V times(double a);
}
