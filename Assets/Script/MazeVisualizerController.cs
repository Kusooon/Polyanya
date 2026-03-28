using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class MazeVisualizerController : MonoBehaviour
{
    [Header("Setting")]
    public float worldWidth = 20f;
    public float worldHeight = 14f;
    public int numSeeds = 60;
    public int seed = 0;

    [Header("UI")]
    public Button generateButton;
    public Button solveButton;
    public Button resetButton;

    public Slider speedSlider;
    public Slider sizeSlider;
    public Slider densitySlider;
    public Slider zoomSlider;

    public TMP_Text statusLabel;

    [Header("Colors")]
    public Color walkableColor = new Color(0.88f, 0.88f, 0.88f);
    public Color edgeColor = new Color(0.50f, 0.50f, 0.50f);
    public Color coneColor = new Color(1.00f, 0.80f, 0.20f, 0.28f);
    public Color expandedColor = new Color(0.30f, 0.60f, 1.00f, 0.13f);
    public Color intervalColor = new Color(1.00f, 0.50f, 0.00f);
    public Color pathColor = new Color(0.10f, 0.90f, 0.20f);
    public Color startColor = new Color(0.10f, 0.85f, 0.20f);
    public Color goalColor = new Color(0.90f, 0.15f, 0.15f);

    [Header("Line Widths")]
    public float edgeWidth = 0.04f;
    public float intervalWidth = 0.07f;
    public float pathWidth = 0.10f;
    public float dotRadius = 0.22f;

    NavMeshData mesh;
    PolyanyaPathfinder pathfinder;
    List<PolyanyaPathfinder.SearchNode> steps = new();

    int currentStep = 0;
    bool solved = false;
    bool animating = false;

    float ZoomBorder => zoomSlider != null ? Mathf.Lerp(1.04f, 2.00f, zoomSlider.value) : 1.04f;
    float StepInterval => speedSlider != null ? Mathf.Lerp(0.30f, 0.01f, speedSlider.value) : 0.05f;

    Material glMat;

    // --- CACHED GEOMETRY BUFFERS ---
    // Pre-calculating vertices saves massive CPU/GPU overhead in OnRenderObject
    Vector3[] _cachedPolyVerts;
    Vector3[] _cachedEdgeVerts;
    Vector3[] _cachedDotsVerts;
    Vector3[] _cachedPathVerts;

    // Cone arrays (updated sequentially as we animate)
    Vector3[] _cachedConeTris;
    Vector3[] _cachedIntervalLines;

    void Start()
    {
        // 1. Cap the framerate so we don't cook the GPU
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 120;

        BuildGLMaterial();

        if (generateButton) generateButton.onClick.AddListener(OnGenerate);
        if (solveButton) solveButton.onClick.AddListener(OnSolve);
        if (resetButton) resetButton.onClick.AddListener(OnReset);

        if (zoomSlider != null)
            zoomSlider.onValueChanged.AddListener(_ => ApplyZoom());

        if (sizeSlider != null) sizeSlider.value = worldWidth;
        if (densitySlider != null) densitySlider.value = numSeeds;
        if (zoomSlider != null) zoomSlider.value = 0.25f;

        OnGenerate();
    }

    public async void OnGenerate()
    {
        if (animating) StopAllCoroutines();
        ToggleUI(false);
        SetStatus("Generating Map...");

        seed++;
        ReadSizeSliders();

        // Copy variables for thread safety
        float w = worldWidth;
        float h = worldHeight;
        int n = numSeeds;
        int s = seed;

        // 2. MULTITHREADING: Offload generation to a background thread
        mesh = await Task.Run(() => VoronoiMazeGenerator.Generate(w, h, n, s));

        Camera.main.transform.position = new Vector3(w / 2f, h / 2f, -10f);
        ApplyZoom();
        UpdateScales();

        // Cache geometry once
        BuildStaticBuffers();

        currentStep = 0;
        solved = false;
        animating = false;
        steps = new();
        pathfinder = null;

        SetStatus($"{worldWidth:0} x {worldHeight:0}\n{mesh.polygons.Count} polygons.\nReady.");
        ToggleUI(true);
    }

    public async void OnSolve()
    {
        if (animating) StopAllCoroutines();
        if (mesh == null) return;

        ToggleUI(false);
        SetStatus("Solving...");

        // MULTITHREADING: Offload pathfinding
        pathfinder = new PolyanyaPathfinder(mesh);
        solved = await Task.Run(() => pathfinder.Solve());
        steps = pathfinder.SearchSteps;

        SetStatus(solved ? $"{steps.Count} nodes\n{pathfinder.FinalPath?.Count ?? 0} waypoints" : "No path found");

        // Cache the search visuals
        BuildSearchBuffers();

        StartCoroutine(AnimateSearch());
        ToggleUI(true);
    }

    public void OnReset()
    {
        StopAllCoroutines();
        currentStep = 0;
        solved = false;
        animating = false;
        SetStatus("Reset");
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  CACHING / OPTIMIZATION METHODS
    // ═════════════════════════════════════════════════════════════════════════

    void BuildStaticBuffers()
    {
        // Walkable Polygons
        var polyVerts = new List<Vector3>();
        foreach (var poly in mesh.polygons)
        {
            var v = poly.vertices;
            for (int i = 1; i < v.Count - 1; i++)
            {
                polyVerts.Add(W(v[0]));
                polyVerts.Add(W(v[i]));
                polyVerts.Add(W(v[i + 1]));
            }
        }
        _cachedPolyVerts = polyVerts.ToArray();

        // Polygon Outlines (Lines)
        var edgeVerts = new List<Vector3>();
        foreach (var poly in mesh.polygons)
        {
            var v = poly.vertices;
            for (int i = 0; i < v.Count; i++)
            {
                edgeVerts.Add(W(v[i]));
                edgeVerts.Add(W(v[(i + 1) % v.Count]));
            }
        }
        _cachedEdgeVerts = edgeVerts.ToArray();

        // Start & Goal Dots
        var dotVerts = new List<Vector3>();
        dotVerts.AddRange(CacheDot(W(mesh.start), dotRadius));
        dotVerts.AddRange(CacheDot(W(mesh.goal), dotRadius));
        _cachedDotsVerts = dotVerts.ToArray();
    }

    void BuildSearchBuffers()
    {
        // Precalculate all cone triangles
        var coneTris = new List<Vector3>();
        var intervalLines = new List<Vector3>();

        foreach (var n in steps)
        {
            // Cone Triangle
            coneTris.Add(W(n.root));
            coneTris.Add(W(n.intLeft));
            coneTris.Add(W(n.intRight));

            // Interval and Edge Lines (Thickened Quads)
            intervalLines.AddRange(CacheLine(W(n.intLeft), W(n.intRight), intervalWidth));
            intervalLines.AddRange(CacheLine(W(n.root), W(n.intLeft), edgeWidth));
            intervalLines.AddRange(CacheLine(W(n.root), W(n.intRight), edgeWidth));
        }

        _cachedConeTris = coneTris.ToArray();
        _cachedIntervalLines = intervalLines.ToArray();

        // Precalculate Final Path
        var pathVerts = new List<Vector3>();
        if (solved && pathfinder?.FinalPath != null && pathfinder.FinalPath.Count >= 2)
        {
            var path = pathfinder.FinalPath;
            for (int i = 0; i < path.Count - 1; i++)
            {
                pathVerts.AddRange(CacheLine(W(path[i]), W(path[i + 1]), pathWidth));
            }
        }
        _cachedPathVerts = pathVerts.ToArray();
    }

    List<Vector3> CacheLine(Vector3 a, Vector3 b, float width)
    {
        Vector3 perp = Vector3.Cross((b - a).normalized, Vector3.forward) * (width * 0.5f);
        return new List<Vector3>
        {
            a - perp, b - perp, b + perp,
            a - perp, b + perp, a + perp
        };
    }

    List<Vector3> CacheDot(Vector3 center, float radius)
    {
        var verts = new List<Vector3>();
        const int segs = 16;
        float step = Mathf.PI * 2f / segs;
        for (int i = 0; i < segs; i++)
        {
            float a0 = step * i, a1 = step * (i + 1);
            verts.Add(center);
            verts.Add(center + new Vector3(Mathf.Cos(a0), Mathf.Sin(a0)) * radius);
            verts.Add(center + new Vector3(Mathf.Cos(a1), Mathf.Sin(a1)) * radius);
        }
        return verts;
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  RENDERING LOOP
    // ═════════════════════════════════════════════════════════════════════════

    void OnRenderObject()
    {
        if (mesh == null || _cachedPolyVerts == null) return;
        glMat.SetPass(0);

        // 1. Walkable polygon fills
        GL.Begin(GL.TRIANGLES);
        GL.Color(walkableColor);
        for (int i = 0; i < _cachedPolyVerts.Length; i++) GL.Vertex(_cachedPolyVerts[i]);
        GL.End();

        // 2. Expanded cone history
        if (steps.Count > 0 && currentStep > 0 && _cachedConeTris != null)
        {
            GL.Begin(GL.TRIANGLES);
            GL.Color(expandedColor);
            int topTri = Mathf.Min(currentStep, steps.Count) * 3;
            for (int i = 0; i < topTri; i++) GL.Vertex(_cachedConeTris[i]);
            GL.End();
        }

        // 3. Current step cone (brighter) & interval lines
        if (currentStep < steps.Count && _cachedConeTris != null)
        {
            GL.Begin(GL.TRIANGLES);

            // Bright Cone
            GL.Color(coneColor);
            int baseIdx = currentStep * 3;
            GL.Vertex(_cachedConeTris[baseIdx]);
            GL.Vertex(_cachedConeTris[baseIdx + 1]);
            GL.Vertex(_cachedConeTris[baseIdx + 2]);

            // Interval Bars (Stored as quads/triangles)
            GL.Color(intervalColor);
            int lineBaseIdx = currentStep * 18; // 3 lines * 6 verts
            for (int i = 0; i < 18; i++) GL.Vertex(_cachedIntervalLines[lineBaseIdx + i]);

            GL.End();
        }

        // 4. NavMesh outlines
        GL.Begin(GL.LINES);
        GL.Color(edgeColor);
        for (int i = 0; i < _cachedEdgeVerts.Length; i++) GL.Vertex(_cachedEdgeVerts[i]);
        GL.End();

        // 5. Final path
        if (solved && _cachedPathVerts != null && currentStep >= steps.Count)
        {
            GL.Begin(GL.TRIANGLES);
            GL.Color(pathColor);
            for (int i = 0; i < _cachedPathVerts.Length; i++) GL.Vertex(_cachedPathVerts[i]);
            GL.End();
        }

        // 6. Start / Goal dots
        GL.Begin(GL.TRIANGLES);
        int halfDot = _cachedDotsVerts.Length / 2;
        GL.Color(startColor);
        for (int i = 0; i < halfDot; i++) GL.Vertex(_cachedDotsVerts[i]);
        GL.Color(goalColor);
        for (int i = halfDot; i < _cachedDotsVerts.Length; i++) GL.Vertex(_cachedDotsVerts[i]);
        GL.End();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  UTILITIES
    // ═════════════════════════════════════════════════════════════════════════

    IEnumerator AnimateSearch()
    {
        animating = true;
        currentStep = 0;
        for (int i = 0; i < steps.Count; i++)
        {
            currentStep = i;
            yield return new WaitForSeconds(StepInterval);
        }
        currentStep = steps.Count;
        animating = false;
        SetStatus(solved ? "Found Path" : "No Path Found");
    }

    void ReadSizeSliders()
    {
        if (sizeSlider != null)
        {
            worldWidth = sizeSlider.value;
            worldHeight = worldWidth * 0.7f;
        }
        if (densitySlider != null)
            numSeeds = Mathf.RoundToInt(densitySlider.value);
    }

    void ApplyZoom()
    {
        if (Camera.main == null || mesh == null) return;
        float aspect = (float)Screen.width / Screen.height;
        float fitByH = worldHeight / 2f;
        float fitByW = worldWidth / (2f * aspect);
        Camera.main.orthographicSize = Mathf.Max(fitByH, fitByW) * ZoomBorder;
    }

    void UpdateScales()
    {
        float scale = worldWidth / 20f;
        edgeWidth = 0.04f * scale;
        intervalWidth = 0.07f * scale;
        pathWidth = 0.10f * scale;
        dotRadius = 0.22f * scale;
    }

    Vector3 W(Vector2 p) => new Vector3(p.x, p.y, 0f);

    void ToggleUI(bool state)
    {
        if (generateButton) generateButton.interactable = state;
        if (solveButton) solveButton.interactable = state;
    }

    void SetStatus(string msg) { if (statusLabel) statusLabel.text = msg; }

    void BuildGLMaterial()
    {
        glMat = new Material(Shader.Find("Hidden/Internal-Colored"));
        glMat.hideFlags = HideFlags.HideAndDontSave;
        glMat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
        glMat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        glMat.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
        glMat.SetInt("_ZWrite", 0);
    }
}