package io.stillflow.runner

import kotlin.math.log2
import kotlin.math.pow

/** Exact aggregates and bounded percentile estimates for one timing stage. */
internal class TimingStatistics {
    companion object {
        const val HISTOGRAM_MIN_MS = 0.000001
        const val HISTOGRAM_BINS_PER_OCTAVE = 64
        const val HISTOGRAM_BINS = 4096
    }

    var count = 0L
        private set
    var total = 0.0
        private set
    var minimum = 0.0
        private set
    var maximum = 0.0
        private set
    private val buckets = LongArray(HISTOGRAM_BINS + 2)

    fun add(ms: Double) {
        if (!ms.isFinite() || ms < 0) return
        val bucket = if (ms < HISTOGRAM_MIN_MS) 0 else {
            val position = log2(ms / HISTOGRAM_MIN_MS) * HISTOGRAM_BINS_PER_OCTAVE
            if (position >= HISTOGRAM_BINS) HISTOGRAM_BINS + 1 else 1 + position.toInt()
        }
        buckets[bucket]++
        minimum = if (count == 0L) ms else minOf(minimum, ms)
        maximum = if (count == 0L) ms else maxOf(maximum, ms)
        total += ms
        count++
    }

    fun percentile(fraction: Double): Double {
        if (count == 0L) return 0.0
        if (fraction <= 0) return minimum
        if (fraction >= 1) return maximum
        val rank = (fraction * (count - 1)).toLong()
        var seen = 0L
        for (bucket in buckets.indices) {
            seen += buckets[bucket]
            if (seen > rank) {
                val midpoint = when (bucket) {
                    0 -> HISTOGRAM_MIN_MS / 2
                    HISTOGRAM_BINS + 1 -> maximum
                    else -> HISTOGRAM_MIN_MS * 2.0.pow((bucket - 0.5) / HISTOGRAM_BINS_PER_OCTAVE)
                }
                return midpoint.coerceIn(minimum, maximum)
            }
        }
        return maximum
    }
}
