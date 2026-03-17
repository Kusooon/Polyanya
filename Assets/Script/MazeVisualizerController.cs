using System.Collections;
using System.Collections.Generic;
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


    void Start()
    {
        BuildGLMaterial();

        if (generateButton) generateButton.onClick.AddListener(OnGenerate);
        if (solveButton) solveButton.onClick.AddListener(OnSolve);
        if (resetButton) resetButton.onClick.AddListener(OnReset);

        if (zoomSlider != null)
            zoomSlider.onValueChanged.AddListener(_ => ApplyZoom());

        if (sizeSlider != null) sizeSlider.value = worldWidth;
        if (densitySlider != null) densitySlider.value = numSeeds;
        if (zoomSlider != null) zoomSlider.value = 0.25f;

        Generate();
    }


    public void OnGenerate()
    {
        seed++;
        ReadSizeSliders();
        Generate();
    }

    public void OnSolve()
    {
        if (animating) StopAllCoroutines();

        pathfinder = new PolyanyaPathfinder(mesh);
        bool found = pathfinder.Solve();
        steps = pathfinder.SearchSteps;

        SetStatus(found ? $"{steps.Count} nodes\n{pathfinder.FinalPath?.Count ?? 0} waypoints": "No path found");

        StartCoroutine(AnimateSearch());
    }

    public void OnReset()
    {
        StopAllCoroutines();
        currentStep = 0;
        solved = false;
        animating = false;
        steps = new();
        pathfinder = null;
        SetStatus("Reset");
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


    void Generate()
    {
        StopAllCoroutines();
        currentStep = 0;
        solved = false;
        animating = false;
        steps = new();
        pathfinder = null;

        mesh = VoronoiMazeGenerator.Generate(worldWidth, worldHeight, numSeeds, seed);

        Camera.main.transform.position =
            new Vector3(worldWidth / 2f, worldHeight / 2f, -10f);

        ApplyZoom();

        float scale = worldWidth / 20f;
        edgeWidth = 0.04f * scale;
        intervalWidth = 0.07f * scale;
        pathWidth = 0.10f * scale;
        dotRadius = 0.22f * scale;

        SetStatus($"{worldWidth:0} x {worldHeight:0}" + $"\n{mesh.polygons.Count} polygons.\nReady.");
    }


    IEnumerator AnimateSearch()
    {
        animating = true;
        currentStep = 0;
        solved = false;

        for (int i = 0; i < steps.Count; i++)
        {
            currentStep = i;
            yield return new WaitForSeconds(StepInterval);
        }

        currentStep = steps.Count;
        solved = pathfinder?.FinalPath != null;
        animating = false;
        SetStatus(solved ? "Found Path" : "No Path Found");
    }


    void OnRenderObject()
    {
        if (mesh == null) return;
        glMat.SetPass(0);

        // walkable polygon fills
        GL.Begin(GL.TRIANGLES);
        GL.Color(walkableColor);
        foreach (var poly in mesh.polygons)
        {
            var v = poly.vertices;
            for (int i = 1; i < v.Count - 1; i++)
            { GL.Vertex(W(v[0])); GL.Vertex(W(v[i])); GL.Vertex(W(v[i + 1])); }
        }
        GL.End();

        // expanded cone history (translucent)
        if (steps.Count > 0 && currentStep > 0)
        {
            GL.Begin(GL.TRIANGLES);
            GL.Color(expandedColor);
            int top = Mathf.Min(currentStep, steps.Count - 1);
            for (int s = 0; s <= top; s++)
            {
                var n = steps[s];
                GL.Vertex(W(n.root));
                GL.Vertex(W(n.intLeft));
                GL.Vertex(W(n.intRight));
            }
            GL.End();
        }

        // current step cone (brighter) + interval bar + cone edges
        if (currentStep < steps.Count)
        {
            var n = steps[currentStep];
            GL.Begin(GL.TRIANGLES);
            GL.Color(coneColor);
            GL.Vertex(W(n.root));
            GL.Vertex(W(n.intLeft));
            GL.Vertex(W(n.intRight));
            GL.End();

            DrawLine(W(n.intLeft), W(n.intRight), intervalWidth, intervalColor);
            DrawLine(W(n.root), W(n.intLeft), edgeWidth, intervalColor);
            DrawLine(W(n.root), W(n.intRight), edgeWidth, intervalColor);
        }

        // navMesh polygon outlines
        GL.Begin(GL.LINES);
        GL.Color(edgeColor);
        foreach (var poly in mesh.polygons)
        {
            var v = poly.vertices;
            for (int i = 0; i < v.Count; i++)
            { GL.Vertex(W(v[i])); GL.Vertex(W(v[(i + 1) % v.Count])); }
        }
        GL.End();

        // final path
        if (solved && pathfinder?.FinalPath != null && pathfinder.FinalPath.Count >= 2)
        {
            var path = pathfinder.FinalPath;
            for (int i = 0; i < path.Count - 1; i++)
                DrawLine(W(path[i]), W(path[i + 1]), pathWidth, pathColor);
        }

        //start (green) and goal (red) dots
        DrawDot(W(mesh.start), dotRadius, startColor);
        DrawDot(W(mesh.goal), dotRadius, goalColor);
    }


    Vector3 W(Vector2 p) => new Vector3(p.x, p.y, 0f);

    void DrawLine(Vector3 a, Vector3 b, float width, Color col)
    {
        Vector3 perp = Vector3.Cross((b - a).normalized, Vector3.forward) * (width * 0.5f);
        GL.Begin(GL.TRIANGLES);
        GL.Color(col);
        GL.Vertex(a - perp); GL.Vertex(b - perp); GL.Vertex(b + perp);
        GL.Vertex(a - perp); GL.Vertex(b + perp); GL.Vertex(a + perp);
        GL.End();
    }

    void DrawDot(Vector3 center, float radius, Color col)
    {
        const int segs = 16;
        float step = Mathf.PI * 2f / segs;
        GL.Begin(GL.TRIANGLES);
        GL.Color(col);
        for (int i = 0; i < segs; i++)
        {
            float a0 = step * i, a1 = step * (i + 1);
            GL.Vertex(center);
            GL.Vertex(center + new Vector3(Mathf.Cos(a0), Mathf.Sin(a0)) * radius);
            GL.Vertex(center + new Vector3(Mathf.Cos(a1), Mathf.Sin(a1)) * radius);
        }
        GL.End();
    }

    void SetStatus(string msg)
    {
        if (statusLabel) statusLabel.text = msg;
    }

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