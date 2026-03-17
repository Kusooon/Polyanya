using System.Collections.Generic;
using System.Linq;
using UnityEngine;


public static class VoronoiMazeGenerator
{
    const float Eps = 2e-4f;
    const float SharedEps = 4e-3f;


    public static NavMeshData Generate(float worldWidth, float worldHeight,
                                        int numSeeds = 50, int seed = 0)
    {
        var rng = new System.Random(seed);


        float mx = worldWidth * 0.06f;
        float my = worldHeight * 0.06f;
        var seeds = new Vector2[numSeeds];
        for (int i = 0; i < numSeeds; i++)
            seeds[i] = new Vector2(
                mx + (float)rng.NextDouble() * (worldWidth - 2 * mx),
                my + (float)rng.NextDouble() * (worldHeight - 2 * my));


        var cellVerts = new List<Vector2>[numSeeds];
        for (int i = 0; i < numSeeds; i++)
            cellVerts[i] = ComputeCell(seeds[i], seeds, worldWidth, worldHeight);


        var adjacency = BuildAdjacency(cellVerts, numSeeds);

        int startIdx = NearestSeed(seeds, numSeeds, new Vector2(0, 0));
        int goalIdx = NearestSeed(seeds, numSeeds, new Vector2(worldWidth, worldHeight));

        if (startIdx == goalIdx || cellVerts[startIdx].Count < 3 || cellVerts[goalIdx].Count < 3)
        {
            var order = Enumerable.Range(0, numSeeds)
                .Where(i => cellVerts[i] != null && cellVerts[i].Count >= 3)
                .OrderBy(i => seeds[i].x + seeds[i].y).ToList();
            startIdx = order.First();
            goalIdx = order.Last();
        }

        var protected_ = new HashSet<int>(
            BFSPath(adjacency, numSeeds, startIdx, goalIdx));

        protected_.Add(startIdx);
        protected_.Add(goalIdx);

        var walkable = new bool[numSeeds];
        for (int i = 0; i < numSeeds; i++)
            walkable[i] = cellVerts[i] != null && cellVerts[i].Count >= 3;

        int targetObstacles = numSeeds * 3 / 10;
        var candidates = Enumerable.Range(0, numSeeds)
            .Where(i => walkable[i] && !protected_.Contains(i))
            .OrderBy(_ => rng.Next())
            .ToList();

        foreach (int idx in candidates)
        {
            if (targetObstacles <= 0) break;
            walkable[idx] = false;

            if (!IsReachable(adjacency, walkable, startIdx, goalIdx))
                walkable[idx] = true;
            else
                targetObstacles--;
        }

        var mesh = new NavMeshData { worldWidth = worldWidth, worldHeight = worldHeight };
        var polys = new NavPolygon[numSeeds];

        for (int i = 0; i < numSeeds; i++)
        {
            if (!walkable[i]) continue;
            var poly = new NavPolygon(i)
            {
                vertices = EnsureCCW(cellVerts[i]),
                centroid = seeds[i]
            };
            polys[i] = poly;
            mesh.polygons.Add(poly);
        }

        for (int i = 0; i < numSeeds; i++)
        {
            if (polys[i] == null) continue;
            foreach (int j in adjacency[i])
            {
                if (j <= i || polys[j] == null) continue;
                if (!FindSharedEdge(cellVerts[i], cellVerts[j], out Vector2 eA, out Vector2 eB))
                    continue;

                var edge = new NavEdge { a = eA, b = eB, polyA = polys[i], polyB = polys[j] };
                polys[i].sharedEdges.Add(edge);
                polys[j].sharedEdges.Add(edge);
            }
        }

        mesh.startPoly = polys[startIdx];
        mesh.goalPoly = polys[goalIdx];
        mesh.start = seeds[startIdx];
        mesh.goal = seeds[goalIdx];

        if (!NavEdgeReachable(mesh.startPoly, mesh.goalPoly))
        {

            RestoreSpineEdges(polys, cellVerts, adjacency, protected_, startIdx, goalIdx);

            if (!NavEdgeReachable(mesh.startPoly, mesh.goalPoly))
                ForceSpineEdges(polys, protected_, startIdx, goalIdx, adjacency);
        }


        return mesh;
    }

    static List<int> BFSPath(List<int>[] adj, int n, int src, int dst)
    {
        var prev = new int[n];
        for (int i = 0; i < n; i++) prev[i] = -1;
        prev[src] = src;

        var queue = new Queue<int>();
        queue.Enqueue(src);

        while (queue.Count > 0)
        {
            int cur = queue.Dequeue();
            if (cur == dst) break;
            foreach (int nb in adj[cur])
                if (prev[nb] < 0) { prev[nb] = cur; queue.Enqueue(nb); }
        }

        if (prev[dst] < 0) return new List<int>();

        var path = new List<int>();
        for (int c = dst; c != src; c = prev[c]) path.Add(c);
        path.Add(src);
        path.Reverse();
        return path;
    }
    static bool IsReachable(List<int>[] adj, bool[] walkable, int src, int dst)
    {
        if (!walkable[src] || !walkable[dst]) return false;
        var visited = new HashSet<int> { src };
        var queue = new Queue<int>();
        queue.Enqueue(src);
        while (queue.Count > 0)
        {
            int cur = queue.Dequeue();
            if (cur == dst) return true;
            foreach (int nb in adj[cur])
                if (walkable[nb] && visited.Add(nb))
                    queue.Enqueue(nb);
        }
        return false;
    }

    static bool NavEdgeReachable(NavPolygon src, NavPolygon dst)
    {
        if (src == null || dst == null) return false;
        if (src == dst) return true;

        var visited = new HashSet<NavPolygon> { src };
        var queue = new Queue<NavPolygon>();
        queue.Enqueue(src);

        while (queue.Count > 0)
        {
            var cur = queue.Dequeue();
            foreach (var edge in cur.sharedEdges)
            {
                var nb = edge.OtherPoly(cur);
                if (nb == null) continue;
                if (nb == dst) return true;
                if (visited.Add(nb)) queue.Enqueue(nb);
            }
        }
        return false;
    }

    static void RestoreSpineEdges(NavPolygon[] polys, List<Vector2>[] cellVerts,
                                   List<int>[] adj, HashSet<int> spine,
                                   int startIdx, int goalIdx)
    {
        var spineList = spine.ToList();
        for (int si = 0; si < spineList.Count; si++)
        {
            int i = spineList[si];
            if (polys[i] == null) continue;
            foreach (int j in adj[i])
            {
                if (j <= i || polys[j] == null || !spine.Contains(j)) continue;

                bool already = polys[i].sharedEdges.Any(
                    e => (e.polyA == polys[i] && e.polyB == polys[j]) ||
                         (e.polyA == polys[j] && e.polyB == polys[i]));
                if (already) continue;

                if (FindSharedEdgeLoose(cellVerts[i], cellVerts[j], out Vector2 eA, out Vector2 eB))
                {
                    var edge = new NavEdge { a = eA, b = eB, polyA = polys[i], polyB = polys[j] };
                    polys[i].sharedEdges.Add(edge);
                    polys[j].sharedEdges.Add(edge);
                }
            }
        }
    }

    static void ForceSpineEdges(NavPolygon[] polys, HashSet<int> spine,
                                 int startIdx, int goalIdx, List<int>[] adj)
    {
        var spineAdj = new List<int>[polys.Length];
        for (int i = 0; i < polys.Length; i++) spineAdj[i] = new List<int>();
        foreach (int i in spine)
            foreach (int j in adj[i])
                if (spine.Contains(j)) { spineAdj[i].Add(j); spineAdj[j].Add(i); }

        var path = BFSPath(spineAdj, polys.Length, startIdx, goalIdx);
        for (int k = 0; k < path.Count - 1; k++)
        {
            int i = path[k], j = path[k + 1];
            if (polys[i] == null || polys[j] == null) continue;
            bool already = polys[i].sharedEdges.Any(
                e => (e.polyA == polys[i] && e.polyB == polys[j]) ||
                     (e.polyA == polys[j] && e.polyB == polys[i]));
            if (already) continue;

            Vector2 mid = (polys[i].centroid + polys[j].centroid) * 0.5f;
            Vector2 perp = Vector2.Perpendicular(
                (polys[j].centroid - polys[i].centroid).normalized) * 0.1f;
            var edge = new NavEdge
            {
                a = mid - perp,
                b = mid + perp,
                polyA = polys[i],
                polyB = polys[j]
            };
            polys[i].sharedEdges.Add(edge);
            polys[j].sharedEdges.Add(edge);
        }
    }

    static List<Vector2> ComputeCell(Vector2 site, Vector2[] allSeeds, float W, float H)
    {
        var poly = new List<Vector2> { new(0, 0), new(W, 0), new(W, H), new(0, H) };
        foreach (var other in allSeeds)
        {
            if (other == site) continue;
            poly = ClipByBisector(poly, site, other);
            if (poly.Count < 3) return new List<Vector2>();
        }
        return poly;
    }

    static List<Vector2> ClipByBisector(List<Vector2> poly, Vector2 site, Vector2 other)
    {
        Vector2 mid = (site + other) * 0.5f;
        Vector2 normal = site - other;
        var result = new List<Vector2>(poly.Count + 1);
        int n = poly.Count;
        for (int i = 0; i < n; i++)
        {
            Vector2 cur = poly[i];
            Vector2 next = poly[(i + 1) % n];
            float dc = Vector2.Dot(cur - mid, normal);
            float dn = Vector2.Dot(next - mid, normal);
            if (dc >= -Eps) result.Add(cur);
            if ((dc < 0) != (dn < 0))
                result.Add(Vector2.Lerp(cur, next, dc / (dc - dn)));
        }
        return result;
    }

    static List<int>[] BuildAdjacency(List<Vector2>[] cells, int n)
    {
        var adj = new List<int>[n];
        for (int i = 0; i < n; i++) adj[i] = new List<int>();
        for (int i = 0; i < n - 1; i++)
        {
            if (cells[i] == null || cells[i].Count < 3) continue;
            for (int j = i + 1; j < n; j++)
            {
                if (cells[j] == null || cells[j].Count < 3) continue;
                if (ShareEdge(cells[i], cells[j]))
                { adj[i].Add(j); adj[j].Add(i); }
            }
        }
        return adj;
    }

    static bool ShareEdge(List<Vector2> a, List<Vector2> b)
    {
        int count = 0;
        foreach (var va in a)
            foreach (var vb in b)
                if (Vector2.Distance(va, vb) < SharedEps)
                    if (++count >= 2) return true;
        return false;
    }

    static bool FindSharedEdge(List<Vector2> a, List<Vector2> b,
                                out Vector2 eA, out Vector2 eB)
    {
        var shared = new List<Vector2>(4);
        foreach (var va in a)
            foreach (var vb in b)
                if (Vector2.Distance(va, vb) < SharedEps)
                    shared.Add(va);
        if (shared.Count >= 2) { eA = shared[0]; eB = shared[1]; return true; }
        eA = eB = Vector2.zero;
        return false;
    }

    static bool FindSharedEdgeLoose(List<Vector2> a, List<Vector2> b,
                                     out Vector2 eA, out Vector2 eB)
    {
        var shared = new List<Vector2>(4);
        float looseTol = SharedEps * 10f;
        foreach (var va in a)
            foreach (var vb in b)
                if (Vector2.Distance(va, vb) < looseTol)
                    shared.Add(va);
        if (shared.Count >= 2) { eA = shared[0]; eB = shared[1]; return true; }
        eA = eB = Vector2.zero;
        return false;
    }


    static int NearestSeed(Vector2[] seeds, int n, Vector2 target)
    {
        int best = 0;
        float bestD = float.MaxValue;
        for (int i = 0; i < n; i++)
        {
            float d = Vector2.Distance(seeds[i], target);
            if (d < bestD) { bestD = d; best = i; }
        }
        return best;
    }

    static List<Vector2> EnsureCCW(List<Vector2> verts)
    {
        float area = 0f;
        for (int i = 0; i < verts.Count; i++)
        {
            Vector2 a = verts[i];
            Vector2 b = verts[(i + 1) % verts.Count];
            area += (b.x - a.x) * (b.y + a.y);
        }
        if (area > 0) { var c = new List<Vector2>(verts); c.Reverse(); return c; }
        return verts;
    }
}