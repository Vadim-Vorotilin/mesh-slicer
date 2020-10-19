using System.Collections.Generic;
using UnityEngine;

public class MeshSlicer : MonoBehaviour {
    
    private struct Intersection {
        public float Enter;
        public Vector3 Point;
    }

    private const float Sigma = 1e-6f;

    [SerializeField] private Mesh _mesh;
    [SerializeField] private Transform _plane;
    [SerializeField] private MeshFilter _target1;
    [SerializeField] private MeshFilter _target2;

    public void Slice() {
        var originalTris = _mesh.triangles;
        var originalVerts = _mesh.vertices;
        
        var newTris = new List<int>(originalTris);
        var newVerts = new List<Vector3>(originalVerts);
        var plane = new Plane(_plane.forward, _plane.position);

        var edgeVerts = new HashSet<int>();

        for (int i = 0; i < originalTris.Length / 3; i++) {
            var isct1 = GetIntersection(plane, originalVerts[originalTris[i * 3]], originalVerts[originalTris[i * 3 + 1]]);
            var isct2 = GetIntersection(plane, originalVerts[originalTris[i * 3 + 1]], originalVerts[originalTris[i * 3 + 2]]);
            var isct3 = GetIntersection(plane, originalVerts[originalTris[i * 3 + 2]], originalVerts[originalTris[i * 3]]);

            int isctCount = (isct1.HasValue ? 1 : 0) +
                            (isct2.HasValue ? 1 : 0) +
                            (isct3.HasValue ? 1 : 0);
            
            if (isctCount == 0)
                continue;

            if (isctCount == 2) {
                Process2Intersections(i, originalTris, isct1, isct2, isct3, ref newVerts, ref newTris, ref edgeVerts);
                continue;
            }

            if (isctCount == 3) {
                Process3Intersections(i, originalVerts, originalTris, isct1, isct2, isct3, ref newVerts, ref newTris, ref edgeVerts);
            }
        }

        HashSet<int> vertsIdx1;
        HashSet<int> vertsIdx2;
        
        DivideOntoTwoSubsets(newVerts, edgeVerts, plane, out vertsIdx1, out vertsIdx2);

        Mesh mesh1;
        Mesh mesh2;
        
        DivideOntoTwoMeshes(newVerts, newTris, vertsIdx1, vertsIdx2, out mesh1, out mesh2);

        _target1.mesh = mesh1;
        _target2.mesh = mesh2;
    }

    private static void DivideOntoTwoMeshes(List<Vector3> verts, List<int> tris,
                                            HashSet<int> vertsIdx1, HashSet<int> vertsIdx2,
                                            out Mesh mesh1, out Mesh mesh2) {
        var verts1 = new List<Vector3>();
        var tris1 = new List<int>();
        var verts2 = new List<Vector3>();
        var tris2 = new List<int>();

        var vToV1 = new Dictionary<int, int>();
        var vToV2 = new Dictionary<int, int>();

        for (int i = 0; i < verts.Count; i++) {
            if (vertsIdx1.Contains(i)) {
                verts1.Add(verts[i]);
                vToV1[i] = verts1.Count - 1;
            }
            
            if (vertsIdx2.Contains(i)) {
                verts2.Add(verts[i]);
                vToV2[i] = verts2.Count - 1;
            }
        }
        
        for (int i = 0; i < tris.Count / 3; i++) {
            if (vertsIdx1.Contains(tris[i * 3]) &&
                vertsIdx1.Contains(tris[i * 3 + 1]) &&
                vertsIdx1.Contains(tris[i * 3 + 2])) {
                
                tris1.Add(vToV1[tris[i * 3]]);
                tris1.Add(vToV1[tris[i * 3 + 1]]);
                tris1.Add(vToV1[tris[i * 3 + 2]]);
            } else if (vertsIdx2.Contains(tris[i * 3]) &&
                       vertsIdx2.Contains(tris[i * 3 + 1]) &&
                       vertsIdx2.Contains(tris[i * 3 + 2])) {
                tris2.Add(vToV2[tris[i * 3]]);
                tris2.Add(vToV2[tris[i * 3 + 1]]);
                tris2.Add(vToV2[tris[i * 3 + 2]]);
            }
        }

        mesh1 = new Mesh {
            vertices = verts1.ToArray(),
            triangles = tris1.ToArray()
        };
        
        mesh1.RecalculateNormals();
        
        mesh2 = new Mesh {
            vertices = verts2.ToArray(),
            triangles = tris2.ToArray()
        };
        
        mesh2.RecalculateNormals();
    }

    private static void DivideOntoTwoSubsets(List<Vector3> verts, HashSet<int> edgeVerts, Plane plane,
                                             out HashSet<int> subsetP, out HashSet<int> subsetN) {
        subsetP = new HashSet<int>(edgeVerts);
        subsetN = new HashSet<int>(edgeVerts);

        for (int i = 0; i < verts.Count; i++) {
            if (edgeVerts.Contains(i))
                continue;

            var dist = plane.GetDistanceToPoint(verts[i]);
            
            if (dist > Sigma)
                subsetP.Add(i);
            else if (dist < -Sigma)
                subsetN.Add(i);
            else {
                subsetP.Add(i);
                subsetN.Add(i);
            }
        }
    }

    private static void Process2Intersections(int i, int[] originalTris, 
                                              Intersection? isct1, Intersection? isct2, Intersection? isct3,
                                              ref List<Vector3> newVerts, ref List<int> newTris,
                                              ref HashSet<int> edgeVerts) {
        int vert;

        if (!isct1.HasValue)
            vert = 2;
        else if (!isct2.HasValue)
            vert = 0;
        else
            vert = 1;

        newTris[i * 3 + vert] = originalTris[i * 3 + vert];

        switch (vert) {
            case 0:
                newVerts.Add(isct3.Value.Point);
                newVerts.Add(isct1.Value.Point);
                
                newTris[i * 3 + 2] = newVerts.Count - 2;
                newTris[i * 3 + 1] = newVerts.Count - 1;
                
                newTris.Add(originalTris[i * 3 + 2]);
                newTris.Add(newVerts.Count - 2);
                newTris.Add(newVerts.Count - 1);
                
                newTris.Add(originalTris[i * 3 + 2]);
                newTris.Add(newVerts.Count - 1);
                newTris.Add(originalTris[i * 3 + 1]);

                break;
            case 1:
                newVerts.Add(isct1.Value.Point);
                newVerts.Add(isct2.Value.Point);
                
                newTris[i * 3] = newVerts.Count - 2;
                newTris[i * 3 + 2] = newVerts.Count - 1;
                
                newTris.Add(originalTris[i * 3]);
                newTris.Add(newVerts.Count - 2);
                newTris.Add(newVerts.Count - 1);
                
                newTris.Add(originalTris[i * 3]);
                newTris.Add(newVerts.Count - 1);
                newTris.Add(originalTris[i * 3 + 2]);

                break;
            case 2:
                newVerts.Add(isct2.Value.Point);
                newVerts.Add(isct3.Value.Point);
                
                newTris[i * 3 + 1] = newVerts.Count - 2;
                newTris[i * 3] = newVerts.Count - 1;
                
                newTris.Add(originalTris[i * 3 + 1]);
                newTris.Add(newVerts.Count - 2);
                newTris.Add(newVerts.Count - 1);
                
                newTris.Add(originalTris[i * 3 + 1]);
                newTris.Add(newVerts.Count - 1);
                newTris.Add(originalTris[i * 3]);

                break;
        }

        if (newTris[i * 3 + 1] == 17) {
            Debug.DrawLine(Vector3.zero, newVerts[newTris[i * 3]], Color.red, 1000, false);
            Debug.DrawLine(Vector3.zero, newVerts[newTris[i * 3 + 1]], Color.red, 1000, false);
            Debug.DrawLine(Vector3.zero, newVerts[newTris[i * 3 + 2]], Color.red, 1000, false);
        }

        edgeVerts.Add(newVerts.Count - 1);
        edgeVerts.Add(newVerts.Count - 2);
    }

    private static void Process3Intersections(int i, Vector3[] originalVerts, int[] originalTris,
                                              Intersection? isct1, Intersection? isct2, Intersection? isct3,
                                              ref List<Vector3> newVerts, ref List<int> newTris,
                                              ref HashSet<int> edgeVerts) {
        int vert;

        if (isct1.Value.Enter < Sigma)
            vert = 0;
        else if (isct2.Value.Enter < Sigma)
            vert = 1;
        else
            vert = 2;

        switch (vert) {
            case 0:
                newVerts.Add(isct2.Value.Point);

                newTris[i * 3 + 1] = newVerts.Count - 1;
                
                newTris.Add(newVerts.Count - 1);
                newTris.Add(originalTris[i * 3]);
                newTris.Add(originalTris[i * 3 + 1]);
                
                Debug.DrawLine(newVerts[newVerts.Count - 1], originalVerts[originalTris[i * 3 + 1]], Color.red, 1000, false);

                break;
            case 1:
                newVerts.Add(isct3.Value.Point);

                newTris[i * 3 + 2] = newVerts.Count - 1;
                
                newTris.Add(newVerts.Count - 1);
                newTris.Add(originalTris[i * 3 + 1]);
                newTris.Add(originalTris[i * 3 + 2]);
                
                Debug.DrawLine(newVerts[newVerts.Count - 1], originalVerts[originalTris[i * 3 + 2]], Color.green, 1000, false);

                break;
            case 2:
                newVerts.Add(isct1.Value.Point);

                newTris[i * 3 + 1] = newVerts.Count - 1;
                
                newTris.Add(newVerts.Count - 1);
                newTris.Add(originalTris[i * 3 + 2]);
                newTris.Add(originalTris[i * 3]);
                
                Debug.DrawLine(newVerts[newVerts.Count - 1], originalVerts[originalTris[i * 3]], Color.blue, 1000, false);

                break;
        }
        
        edgeVerts.Add(newVerts.Count - 1);
    }

    private static Intersection? GetIntersection(Plane plane, Vector3 p1, Vector3 p2) {
        Ray ray = new Ray(p1, p2 - p1);
        float enter;

        if (!plane.Raycast(ray, out enter))
            return null;

        if (enter * enter > (p1 - p2).sqrMagnitude || enter < 0)
            return null;

        return new Intersection { Enter = enter, Point = ray.GetPoint(enter) };
    }
}