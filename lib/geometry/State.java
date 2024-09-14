package com.uni.lib.geometry;

import com.uni.lib.util.CSVWritable;
import com.uni.lib.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    S add(S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
