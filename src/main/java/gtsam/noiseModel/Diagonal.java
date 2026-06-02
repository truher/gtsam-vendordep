package gtsam.noiseModel;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.Vector;
import gtsam.Vector1;
import gtsam.Vector2;
import gtsam.Vector3;
import gtsam.shared_ptr;

public class Diagonal extends Gaussian {
    public enum FF {
        noiseModel_Diagonal_SigmasVector(ADDRESS, ADDRESS),
        noiseModel_Diagonal_SigmasVector1(ADDRESS, ADDRESS),
        noiseModel_Diagonal_SigmasVector2(ADDRESS, ADDRESS),
        noiseModel_Diagonal_SigmasVector3(ADDRESS, ADDRESS),
        noiseModel_Diagonal_delete(null, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Diagonal(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Diagonal> Sigmas(Vector v) throws Throwable {
        return new shared_ptr<Diagonal>(
                (MemorySegment) FF.noiseModel_Diagonal_SigmasVector.h.invokeExact(v.ptr),
                Diagonal::new,
                FF.noiseModel_Diagonal_delete.h);
    }

    public static shared_ptr<Diagonal> Sigmas(Vector1 v) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.noiseModel_Diagonal_SigmasVector1.h.invokeExact(v.ptr),
                Diagonal::new,
                FF.noiseModel_Diagonal_delete.h);
    }

    public static shared_ptr<Diagonal> Sigmas(Vector2 v) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.noiseModel_Diagonal_SigmasVector2.h.invokeExact(v.ptr),
                Diagonal::new,
                FF.noiseModel_Diagonal_delete.h);
    }

    public static shared_ptr<Diagonal> Sigmas(Vector3 v) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.noiseModel_Diagonal_SigmasVector3.h.invokeExact(v.ptr),
                Diagonal::new,
                FF.noiseModel_Diagonal_delete.h);
    }

}
