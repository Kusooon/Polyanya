using System.Collections.Generic;
using UnityEngine;

public class NavPolygon
{
    public int id;
    public List<Vector2> vertices = new();
    public List<NavEdge> sharedEdges = new();
    public Vector2 centroid;
    public NavPolygon(int id) => this.id = id;
    public bool Contains(Vector2 p)
    {
        for (int i = 0; i < vertices.Count; i++)
        {
            Vector2 a = vertices[i];
            Vector2 b = vertices[(i + 1) % vertices.Count];
            //for CCW winding, p must be on the left of every edge.
            if (Cross2D(b - a, p - a) < -1e-5f) return false;
        }
        return true;
    }

    public static float Cross2D(Vector2 a, Vector2 b) => a.x * b.y - a.y * b.x;
}

public class NavEdge
{
    public Vector2 a, b;
    public NavPolygon polyA;
    public NavPolygon polyB;

    public NavPolygon OtherPoly(NavPolygon p) => p == polyA ? polyB : polyA;

    //returns (left, right) such that the edge reads CCW from inside polygon p.
    public (Vector2 left, Vector2 right) OrderedFor(NavPolygon p)
        => p == polyA ? (a, b) : (b, a);
}

public class NavMeshData
{
    public List<NavPolygon> polygons = new();

    public Vector2 start, goal;
    public NavPolygon startPoly, goalPoly;
    public float worldWidth, worldHeight;

    // Returns the first walkable polygon that contains pt, or null.
    public NavPolygon FindContaining(Vector2 pt)
    {
        foreach (var poly in polygons)
            if (poly.Contains(pt)) return poly;
        return null;
    }
}