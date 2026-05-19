package gtsam;

import java.lang.foreign.MemorySegment;

/**
 * see VectorSpace.h. and VectorSpace.md.
 * 
 * A GTSAM VectorSpace is a trait, which is applied
 * to some common types (e.g. matrices and vectors)
 * 
 *  TODO: include dimension as a type parameter somehow.
 */
public interface VectorSpace<T extends VectorSpace<T>> {
    /** Vector space dimension. */
    int getDimension() throws Throwable;

    Vector local(T other) throws Throwable;

    MemorySegment ptr();
}
