package frc.robot.utils;

import java.util.Iterator;
import java.util.Spliterators;
import java.util.function.BiFunction;
import java.util.stream.Stream;
import java.util.stream.StreamSupport;

import frc.robot.utils.functional.QuadFunction;
import frc.robot.utils.functional.TriFunction;

public class StreamUtils {
    public static <A, B, C> Stream<C> zip(
        Stream<? extends A> a,
        Stream<? extends B> b,
        BiFunction<? super A,? super B,? extends C> zipper
    ){
        Iterator<? extends A> iteratorA = a.iterator();
        Iterator<? extends B> iteratorB = b.iterator();

        Iterator<C> ziperator = new Iterator<>() {
            @Override
            public boolean hasNext() {
                return iteratorA.hasNext() && iteratorB.hasNext();
            }

            @Override
            public C next() {
                return zipper.apply(iteratorA.next(), iteratorB.next());
            }
        };

        return StreamSupport.stream(
            Spliterators.spliteratorUnknownSize(ziperator, 0),
            false
        );
    }

    public static <A, B, C, D> Stream<D> triZip(
        Stream<? extends A> a,
        Stream<? extends B> b,
        Stream<? extends C> c,
        TriFunction<? super A, ? super B, ? super C, ? extends D> triZipper
    ) {
        Iterator<? extends A> iteratorA = a.iterator();
        Iterator<? extends B> iteratorB = b.iterator();
        Iterator<? extends C> iteratorC = c.iterator();

        Iterator<D> triZiperator = new Iterator<>() {
            @Override
            public boolean hasNext() {
                return iteratorA.hasNext() && iteratorB.hasNext() && iteratorC.hasNext();
            }

            @Override
            public D next() {
                return triZipper.apply(iteratorA.next(), iteratorB.next(), iteratorC.next());
            }
        };

        return StreamSupport.stream(
            Spliterators.spliteratorUnknownSize(triZiperator, 0),
            false
        );
    }

    public static <A, B, C, D, E> Stream<E> quadZip(
        Stream<? extends A> a,
        Stream<? extends B> b,
        Stream<? extends C> c,
        Stream<? extends D> d,
        QuadFunction<? super A, ? super B, ? super C, ? super D, ? extends E> quadZipper
    ) {
        Iterator<? extends A> iteratorA = a.iterator();
        Iterator<? extends B> iteratorB = b.iterator();
        Iterator<? extends C> iteratorC = c.iterator();
        Iterator<? extends D> iteratorD = d.iterator();

        Iterator<E> quadZiperator = new Iterator<>() {
            @Override
            public boolean hasNext() {
                return iteratorA.hasNext() && iteratorB.hasNext() && iteratorC.hasNext() && iteratorD.hasNext();
            }

            @Override
            public E next() {
                return quadZipper.apply(iteratorA.next(), iteratorB.next(), iteratorC.next(), iteratorD.next());
            }
        };

        return StreamSupport.stream(
            Spliterators.spliteratorUnknownSize(quadZiperator, 0),
            false
        );
    }
}
