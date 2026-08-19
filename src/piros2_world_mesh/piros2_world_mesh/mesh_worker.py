"""
The mesh refresh's heavy half in its own process — the GIL escape hatch.

SLAM plan P3 finding: at 1.5 cm voxels a close-range scene extracts to
0.7-1.6 M triangles, and quadric decimation to the marker budget plus
the completion pass cost 12-21 s. Run inline that starved integration
(measured: ~50 frames of an 88 s bag). Run on a *thread* it still did —
Open3D's decimation holds the GIL, so the executor thread sat blocked
for the same seconds (measured: zero frames integrated for 20 s at a
time; the frame memory of a loop bag came out 90 outbound / 13 return).
A separate interpreter is the only thing that actually frees the node.

`MeshFinisher` owns one long-lived worker process (spawn context — no
CUDA state is inherited): the node hands it arrays (pickled through a
pipe, ~30-60 MB, ~100 ms) and later collects the finished arrays with a
non-blocking poll. Only numpy + cv2-free code crosses: open3d is
imported in the child, so this module also stays importable — and unit
testable, the worker included — on the system interpreter as long as
open3d exists there; when it does not, decimation is skipped in the
child and the completion pass still runs.
"""

import multiprocessing as mp
import queue

import numpy as np

from .mesh_fill import complete_mesh


def finish_mesh(vertices, triangles, colours, budget, min_component,
                max_hole_radius, tint):
    """Decimate to budget (if open3d is present) then complete. Pure."""
    if budget and len(triangles) > budget:
        try:
            import open3d as o3d
            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(vertices)
            mesh.triangles = o3d.utility.Vector3iVector(triangles)
            mesh.vertex_colors = o3d.utility.Vector3dVector(colours)
            dec = mesh.simplify_quadric_decimation(
                target_number_of_triangles=int(budget))
            vertices = np.asarray(dec.vertices)
            triangles = np.asarray(dec.triangles)
            colours = np.asarray(dec.vertex_colors)
        except ImportError:
            pass
    vertices, triangles, colours, stats = complete_mesh(
        vertices, triangles, colours, min_component, max_hole_radius, tint)
    return vertices, triangles, colours, stats


def _worker(jobs, results):
    # The finisher is bulk work nobody waits on; the odometry and the
    # integration are latency-bound. Give the CPU to them.
    try:
        import os
        os.nice(10)
    except (OSError, AttributeError):
        pass
    while True:
        job = jobs.get()
        if job is None:
            return
        job_id, args = job
        try:
            results.put((job_id, finish_mesh(*args), None))
        except Exception as exc:  # report, never die silently
            results.put((job_id, None, repr(exc)))


class MeshFinisher:
    """One background process; submit() when idle, poll() for the result."""

    def __init__(self):
        self._ctx = mp.get_context('spawn')
        self._process = None
        self._jobs = None
        self._results = None
        self._pending = None
        self._next_id = 0

    def _ensure(self):
        if self._process is not None and self._process.is_alive():
            return
        self._jobs = self._ctx.Queue()
        self._results = self._ctx.Queue()
        self._process = self._ctx.Process(
            target=_worker, args=(self._jobs, self._results), daemon=True)
        self._process.start()

    @property
    def busy(self):
        return self._pending is not None

    def submit(self, vertices, triangles, colours, budget, min_component,
               max_hole_radius, tint=None):
        """Hand a mesh to the worker; False if one is still in flight."""
        if self.busy:
            return False
        self._ensure()
        self._next_id += 1
        self._pending = self._next_id
        self._jobs.put((self._pending, (
            np.ascontiguousarray(vertices), np.ascontiguousarray(triangles),
            np.ascontiguousarray(colours), budget, min_component,
            max_hole_radius, tint)))
        return True

    def poll(self):
        """
        Return (vertices, triangles, colours, stats) when done, else None.

        Raises RuntimeError with the worker's message if the job failed.
        """
        if not self.busy:
            return None
        try:
            job_id, result, error = self._results.get_nowait()
        except queue.Empty:
            if not self._process.is_alive():
                self._pending = None
                raise RuntimeError('mesh worker died')
            return None
        if job_id != self._pending:
            return None
        self._pending = None
        if error is not None:
            raise RuntimeError(error)
        return result

    def close(self):
        if self._process is not None and self._process.is_alive():
            self._jobs.put(None)
            self._process.join(timeout=2.0)
            if self._process.is_alive():
                self._process.terminate()
        self._process = None
