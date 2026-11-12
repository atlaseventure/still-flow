package io.stillflow

import android.os.Debug
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

/** Bounded analysis worker. Retain input storage until its planned frame is consumed. */
class StreamPlanner(width: Int, height: Int, fps: Double, horizontalScan: Boolean, settings: StillFlow.Settings) : AutoCloseable {
    class CpuProfile internal constructor(@JvmField val analysisMs: Double, @JvmField val planningMs: Double)
    private var analysisCpuNs = 0L
    private var planningCpuNs = 0L
    class PlannedFrame internal constructor(
        @JvmField val index: Long,
        @JvmField val ptsUs: Long,
        @JvmField val durationUs: Long,
        internal val map: FloatArray,
        @JvmField val analysisStartedNs: Long,
        @JvmField val analysisCompletedNs: Long,
        @JvmField val planCompletedNs: Long,
    ) {
        fun copyMap(): FloatArray = map.copyOf()
    }
    private class Input(val frame: StillFlow.Frame?, val pts: Long = 0, val duration: Long = 0)
    private val inputs = ArrayBlockingQueue<Input>(2)
    private val outputs = ArrayBlockingQueue<PlannedFrame>(3)
    private val failure = AtomicReference<Throwable?>()
    private val done = AtomicBoolean()
    private val closing = AtomicBoolean()
    private val profile = AtomicReference("[]")
    private var sealed = false
    private val worker: Thread

    init {
        require(settings.realtime) { "StreamPlanner requires realtime settings" }
        // Create synchronously to snapshot caller-owned settings before starting the worker.
        val session = StillFlow(width, height, fps, horizontalScan, settings)
        worker = Thread({
            try {
                session.use {
                    val analysisTimes = java.util.ArrayDeque<Pair<Long, Long>>()
                    fun drain() {
                        while (true) {
                            val cpuStart = Debug.threadCpuTimeNanos()
                            val ready = session.takeReady()
                            planningCpuNs += Debug.threadCpuTimeNanos() - cpuStart
                            if (ready == null) break
                            val map = session.takeReadyMap()
                            val times = analysisTimes.removeFirst()
                            outputs.put(PlannedFrame(ready.index, ready.ptsUs, ready.durationUs, map, times.first, times.second, System.nanoTime()))
                        }
                    }
                    while (true) {
                        val input = inputs.take()
                        if (input.frame == null) break
                        val start = System.nanoTime()
                        val cpuStart = Debug.threadCpuTimeNanos()
                        check(session.tryPush(input.frame, input.pts, input.duration)) { "Analysis buffer was not drained" }
                        analysisCpuNs += Debug.threadCpuTimeNanos() - cpuStart
                        analysisTimes.addLast(Pair(start, System.nanoTime()))
                        drain()
                    }
                    session.finishStream()
                    drain()
                    profile.set(session.profile())
                }
            } catch (error: Throwable) {
                if (!closing.get()) failure.set(error)
            } finally { done.set(true) }
        }, "stillflow-analysis")
        try { worker.start() } catch (error: Throwable) { session.close(); throw error }
    }

    /** False applies backpressure; consume ready plans and retry this exact frame. Single producer. */
    fun trySubmit(frame: StillFlow.Frame, ptsUs: Long, durationUs: Long): Boolean {
        checkFailure()
        check(!sealed) { "Input is sealed" }
        return inputs.offer(Input(frame, ptsUs, durationUs))
    }

    /** False means consume ready plans before retrying. Single producer. */
    fun tryFinish(): Boolean {
        checkFailure()
        if (sealed) return true
        if (!inputs.offer(Input(null))) return false
        sealed = true
        return true
    }

    @JvmOverloads
    fun takeReady(timeoutMs: Long = 0): PlannedFrame? {
        checkFailure()
        require(timeoutMs >= 0)
        val result = if (timeoutMs == 0L) outputs.poll() else outputs.poll(timeoutMs, TimeUnit.MILLISECONDS)
        checkFailure()
        return result
    }

    val finished: Boolean get() { checkFailure(); return done.get() && outputs.isEmpty() }
    fun profile(): String { check(finished); return profile.get() }
    fun cpuProfile(): CpuProfile { check(finished); return CpuProfile(analysisCpuNs / 1e6, planningCpuNs / 1e6) }
    private fun checkFailure() {
        failure.get()?.let { throw IllegalStateException("Streaming analysis failed", it) }
        check(!closing.get()) { "Planner is closed" }
    }
    override fun close() {
        if (closing.getAndSet(true)) return
        worker.interrupt()
        worker.join()
        inputs.clear()
        outputs.clear()
    }
}
