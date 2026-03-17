using System.Collections.Generic;
using UnityEngine;


public class PolyanyaPathfinder
{

    public class SearchNode
    {
        public Vector2 root;
        public Vector2 intLeft;
        public Vector2 intRight;
        public NavEdge edge;
        public NavPolygon poly;
        public float g;
        public float f;
        public SearchNode parent;
        public bool isGoal;
    }


    public List<SearchNode> SearchSteps { get; } = new();
    public List<Vector2> FinalPath { get; private set; }

    readonly NavMeshData mesh;
    public PolyanyaPathfinder(NavMeshData mesh) => this.mesh = mesh;


    public bool Solve() => Solve(mesh.start, mesh.goal, mesh.startPoly, mesh.goalPoly);

    public bool Solve(Vector2 start, Vector2 goal,
                      NavPolygon startPoly, NavPolygon goalPoly)
    {
        SearchSteps.Clear();
        FinalPath = null;
        if (startPoly == null || goalPoly == null) return false;

        if (startPoly == goalPoly)
        {
            FinalPath = new List<Vector2> { start, goal };
            return true;
        }


        if (RunPolyanya(start, goal, startPoly, goalPoly))
            return true;

        if (NavEdgeReachable(startPoly, goalPoly))
        {
            Debug.LogWarning("[Polyanya] Cone search failed — using edge-midpoint fallback.");
            return EdgeMidpointAStar(start, goal, startPoly, goalPoly);
        }

        return false;
    }

    bool RunPolyanya(Vector2 start, Vector2 goal,
                     NavPolygon startPoly, NavPolygon goalPoly)
    {
        var heap = new MinHeap<SearchNode>((a, b) => a.f.CompareTo(b.f));
        var bestG = new Dictionary<(int, long), float>();

        foreach (var edge in startPoly.sharedEdges)
        {
            var adj = edge.OtherPoly(startPoly);
            if (adj == null) continue;
            (Vector2 el, Vector2 er) = LeftRight(edge.a, edge.b, start);
            PushNode(heap, bestG, null, start, 0f, el, er, edge, adj,
                     adj == goalPoly, goal);
        }

        while (heap.Count > 0)
        {
            var n = heap.Pop();
            var sk = StaleKey(n.edge, n.root);
            if (bestG.TryGetValue(sk, out float rec) && n.g > rec + 1e-5f) continue;

            SearchSteps.Add(n);

            if (n.isGoal)
            {
                Vector2 pStar = ClosestPointOnSegToLine(n.intLeft, n.intRight, n.root, goal);
                FinalPath = BuildPath(n, start, pStar, goal);
                return true;
            }

            foreach (var nextEdge in n.poly.sharedEdges)
            {
                if (nextEdge == n.edge) continue;
                var nextPoly = nextEdge.OtherPoly(n.poly);
                if (nextPoly == null) continue;

                (Vector2 el, Vector2 er) = LeftRight(nextEdge.a, nextEdge.b, n.root);
                var clipped = ClipInterval(el, er, n.root, n.intLeft, n.intRight);
                if (clipped == null) continue;
                (Vector2 cl, Vector2 cr) = clipped.Value;
                if (Dist(cl, cr) < 1e-6f) continue;

                bool nextIsGoal = nextPoly == goalPoly;
                bool clIsVert = Near(cl, el) || Near(cl, er);
                bool crIsVert = Near(cr, el) || Near(cr, er);
                bool leftTurn = clIsVert && Near(cl, n.intLeft);
                bool rightTurn = crIsVert && Near(cr, n.intRight);

                if (leftTurn && rightTurn)
                {
                    PushNode(heap, bestG, n, cl, n.g + Dist(n.root, cl),
                             cl, cr, nextEdge, nextPoly, nextIsGoal, goal);
                    PushNode(heap, bestG, n, cr, n.g + Dist(n.root, cr),
                             cl, cr, nextEdge, nextPoly, nextIsGoal, goal);
                }
                else if (leftTurn)
                    PushNode(heap, bestG, n, cl, n.g + Dist(n.root, cl),
                             cl, cr, nextEdge, nextPoly, nextIsGoal, goal);
                else if (rightTurn)
                    PushNode(heap, bestG, n, cr, n.g + Dist(n.root, cr),
                             cl, cr, nextEdge, nextPoly, nextIsGoal, goal);
                else
                    PushNode(heap, bestG, n, n.root, n.g,
                             cl, cr, nextEdge, nextPoly, nextIsGoal, goal);
            }
        }
        return false;
    }

    void PushNode(
        MinHeap<SearchNode> heap,
        Dictionary<(int, long), float> bestG,
        SearchNode parent,
        Vector2 root, float g,
        Vector2 intL, Vector2 intR,
        NavEdge edge, NavPolygon poly,
        bool isGoal, Vector2 goal)
    {
        var key = StaleKey(edge, root);
        if (bestG.TryGetValue(key, out float old) && g >= old - 1e-5f) return;
        bestG[key] = g;

        float h = isGoal
            ? Dist(root, ClosestPointOnSegToLine(intL, intR, root, goal)) +
              Dist(ClosestPointOnSegToLine(intL, intR, root, goal), goal)
            : HThrough(root, intL, intR, goal);

        heap.Push(new SearchNode
        {
            root = root,
            g = g,
            f = g + h,
            intLeft = intL,
            intRight = intR,
            edge = edge,
            poly = poly,
            parent = parent,
            isGoal = isGoal
        });
    }

    bool EdgeMidpointAStar(Vector2 start, Vector2 goal,
                           NavPolygon startPoly, NavPolygon goalPoly)
    {
        var gScore = new Dictionary<NavPolygon, float> { [startPoly] = 0f };
        var prev = new Dictionary<NavPolygon, (NavPolygon from, Vector2 via)>();
        var heap = new MinHeap<(float f, NavPolygon p)>((a, b) => a.f.CompareTo(b.f));
        heap.Push((Dist(start, goal), startPoly));

        while (heap.Count > 0)
        {
            var (_, cur) = heap.Pop();
            float gCur = gScore[cur];

            if (cur == goalPoly)
            {
                FinalPath = ReconstructMidpointPath(prev, startPoly, goalPoly, start, goal);
                return FinalPath != null && FinalPath.Count >= 2;
            }

            foreach (var edge in cur.sharedEdges)
            {
                var nb = edge.OtherPoly(cur);
                if (nb == null) continue;
                Vector2 mid = (edge.a + edge.b) * 0.5f;
                float gVia = gCur + Dist(cur.centroid, mid) + Dist(mid, nb.centroid);
                if (!gScore.TryGetValue(nb, out float oldG) || gVia < oldG - 1e-5f)
                {
                    gScore[nb] = gVia;
                    prev[nb] = (cur, mid);
                    heap.Push((gVia + Dist(nb.centroid, goal), nb));
                }
            }
        }
        return false;
    }

    List<Vector2> ReconstructMidpointPath(
        Dictionary<NavPolygon, (NavPolygon, Vector2)> prev,
        NavPolygon startPoly, NavPolygon goalPoly,
        Vector2 start, Vector2 goal)
    {
        var mids = new List<Vector2>();
        var cur = goalPoly;
        int safety = 10000;
        while (cur != startPoly && safety-- > 0)
        {
            if (!prev.TryGetValue(cur, out var p)) return null;
            mids.Add(p.Item2);
            cur = p.Item1;
        }
        mids.Reverse();

        var pts = new List<Vector2> { start };
        pts.AddRange(mids);
        pts.Add(goal);
        return StringPull(pts);
    }

    List<Vector2> BuildPath(SearchNode n, Vector2 start, Vector2 pStar, Vector2 goal)
    {
        var pts = new List<Vector2> { goal };
        if (Dist(pStar, goal) > 1e-4f) pts.Add(pStar);
        for (; n != null; n = n.parent)
            if (Dist(n.root, pts[^1]) > 1e-4f)
                pts.Add(n.root);
        if (Dist(start, pts[^1]) > 1e-4f) pts.Add(start);
        pts.Reverse();
        return StringPull(pts);
    }

    static List<Vector2> StringPull(List<Vector2> pts)
    {
        if (pts.Count <= 2) return pts;
        var r = new List<Vector2> { pts[0] };
        for (int i = 1; i < pts.Count - 1; i++)
            if (Mathf.Abs(Cross2D(pts[i] - r[^1], pts[i + 1] - r[^1])) > 1e-4f)
                r.Add(pts[i]);
        r.Add(pts[^1]);
        return r;
    }

    static float HThrough(Vector2 root, Vector2 intL, Vector2 intR, Vector2 goal)
    {
        if (SegmentsIntersect(root, goal, intL, intR))
            return Dist(root, goal);
        return Mathf.Min(Dist(root, intL) + Dist(intL, goal),
                         Dist(root, intR) + Dist(intR, goal));
    }

    static (Vector2, Vector2)? ClipInterval(
        Vector2 a, Vector2 b,
        Vector2 apex, Vector2 coneL, Vector2 coneR)
    {
        float tMin = 0f, tMax = 1f;

        {
            Vector2 d = coneL - apex;
            float fa = Cross2D(d, a - apex);
            float fb = Cross2D(d, b - apex);
            float fr = Cross2D(d, coneR - apex);
            if (Mathf.Abs(fr) > 1e-9f)
            {
                if (fr < 0) { fa = -fa; fb = -fb; }
                if (!Clip1D(fa, fb, ref tMin, ref tMax)) return null;
            }
        }

        {
            Vector2 d = coneR - apex;
            float fa = Cross2D(d, a - apex);
            float fb = Cross2D(d, b - apex);
            float fl = Cross2D(d, coneL - apex);
            if (Mathf.Abs(fl) > 1e-9f)
            {
                if (fl < 0) { fa = -fa; fb = -fb; }
                if (!Clip1D(fa, fb, ref tMin, ref tMax)) return null;
            }
        }

        if (tMin > tMax + 1e-6f) return null;
        return (Vector2.Lerp(a, b, tMin), Vector2.Lerp(a, b, tMax));
    }

    static bool Clip1D(float fa, float fb, ref float tMin, ref float tMax)
    {
        if (fa >= -1e-7f && fb >= -1e-7f) return true;
        if (fa < -1e-7f && fb < -1e-7f) return false;
        float t = fa / (fa - fb);
        if (fa < 0) tMin = Mathf.Max(tMin, t);
        else tMax = Mathf.Min(tMax, t);
        return tMin <= tMax + 1e-6f;
    }

    static Vector2 ClosestPointOnSegToLine(Vector2 a, Vector2 b, Vector2 p, Vector2 q)
    {
        Vector2 ab = b - a, pq = q - p;
        float denom = Cross2D(ab, pq);
        if (Mathf.Abs(denom) < 1e-9f)
        {
            return Dist(p, a) + Dist(a, q) <= Dist(p, b) + Dist(b, q) ? a : b;
        }
        float t = Mathf.Clamp01(Cross2D(p - a, pq) / denom);
        return Vector2.Lerp(a, b, t);
    }

    static (Vector2, Vector2) LeftRight(Vector2 a, Vector2 b, Vector2 root)
        => Cross2D(a - root, b - root) <= 0 ? (a, b) : (b, a);

    static bool SegmentsIntersect(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4)
    {
        float d1 = Cross2D(p4 - p3, p1 - p3), d2 = Cross2D(p4 - p3, p2 - p3);
        float d3 = Cross2D(p2 - p1, p3 - p1), d4 = Cross2D(p2 - p1, p4 - p1);
        if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) return true;
        if (Mathf.Abs(d1) < 1e-6f && OnSeg(p3, p4, p1)) return true;
        if (Mathf.Abs(d2) < 1e-6f && OnSeg(p3, p4, p2)) return true;
        if (Mathf.Abs(d3) < 1e-6f && OnSeg(p1, p2, p3)) return true;
        if (Mathf.Abs(d4) < 1e-6f && OnSeg(p1, p2, p4)) return true;
        return false;
    }

    static bool OnSeg(Vector2 a, Vector2 b, Vector2 p) =>
        Mathf.Min(a.x, b.x) - 1e-6f <= p.x && p.x <= Mathf.Max(a.x, b.x) + 1e-6f &&
        Mathf.Min(a.y, b.y) - 1e-6f <= p.y && p.y <= Mathf.Max(a.y, b.y) + 1e-6f;

    static bool NavEdgeReachable(NavPolygon src, NavPolygon dst)
    {
        if (src == null || dst == null) return false;
        if (src == dst) return true;
        var visited = new HashSet<NavPolygon> { src };
        var q = new Queue<NavPolygon>();
        q.Enqueue(src);
        while (q.Count > 0)
        {
            var cur = q.Dequeue();
            foreach (var e in cur.sharedEdges)
            {
                var nb = e.OtherPoly(cur);
                if (nb == dst) return true;
                if (nb != null && visited.Add(nb)) q.Enqueue(nb);
            }
        }
        return false;
    }

    static (int, long) StaleKey(NavEdge e, Vector2 root) =>
        (e.GetHashCode(),
         (long)(Mathf.Round(root.x * 200f)) * 1_000_000L +
         (long)(Mathf.Round(root.y * 200f)));

    static float Cross2D(Vector2 a, Vector2 b) => a.x * b.y - a.y * b.x;
    static float Dist(Vector2 a, Vector2 b) => Vector2.Distance(a, b);
    static bool Near(Vector2 a, Vector2 b) => Vector2.Distance(a, b) < 1e-4f;
}

public class MinHeap<T>
{
    readonly List<T> data = new();
    readonly System.Comparison<T> compare;

    public int Count => data.Count;
    public MinHeap(System.Comparison<T> cmp) => compare = cmp;

    public void Push(T item) { data.Add(item); BubbleUp(data.Count - 1); }

    public T Pop()
    {
        T top = data[0];
        int last = data.Count - 1;
        data[0] = data[last]; data.RemoveAt(last);
        if (data.Count > 0) SiftDown(0);
        return top;
    }

    void BubbleUp(int i)
    {
        while (i > 0)
        {
            int p = (i - 1) / 2;
            if (compare(data[i], data[p]) < 0) { Swap(i, p); i = p; } else break;
        }
    }

    void SiftDown(int i)
    {
        int n = data.Count;
        while (true)
        {
            int l = 2 * i + 1, r = 2 * i + 2, s = i;
            if (l < n && compare(data[l], data[s]) < 0) s = l;
            if (r < n && compare(data[r], data[s]) < 0) s = r;
            if (s == i) break;
            Swap(i, s); i = s;
        }
    }

    void Swap(int a, int b) => (data[a], data[b]) = (data[b], data[a]);
}