#include "sps/core/mesh.h"
#include "sps/io/mesh_io.h"
#include "sps/filter/bilateral_filter.h"
#include "sps/detection/region_detection.h"
#include "sps/simplify/qem_simplifier.h"
#include <iostream>
#include <cassert>

using namespace sps;

void testMeshBasic() {
    std::cout << "Test: Mesh basic operations... ";

    Mesh mesh;

    // Create a simple triangle
    mesh.addVertex(Vector3d(0, 0, 0));
    mesh.addVertex(Vector3d(1, 0, 0));
    mesh.addVertex(Vector3d(0, 1, 0));
    mesh.addFace(0, 1, 2);

    assert(mesh.vertexCount() == 3);
    assert(mesh.faceCount() == 1);
    assert(mesh.faces[0].area > 0);

    std::cout << "PASSED\n";
}

void testMeshCube() {
    std::cout << "Test: Cube mesh... ";

    Mesh mesh;

    // Create cube vertices
    mesh.addVertex(Vector3d(0, 0, 0));
    mesh.addVertex(Vector3d(1, 0, 0));
    mesh.addVertex(Vector3d(1, 1, 0));
    mesh.addVertex(Vector3d(0, 1, 0));
    mesh.addVertex(Vector3d(0, 0, 1));
    mesh.addVertex(Vector3d(1, 0, 1));
    mesh.addVertex(Vector3d(1, 1, 1));
    mesh.addVertex(Vector3d(0, 1, 1));

    // Create cube faces (2 triangles per face)
    // Bottom
    mesh.addFace(0, 1, 2);
    mesh.addFace(0, 2, 3);
    // Top
    mesh.addFace(4, 6, 5);
    mesh.addFace(4, 7, 6);
    // Front
    mesh.addFace(0, 5, 1);
    mesh.addFace(0, 4, 5);
    // Back
    mesh.addFace(2, 7, 3);
    mesh.addFace(2, 6, 7);
    // Left
    mesh.addFace(0, 3, 7);
    mesh.addFace(0, 7, 4);
    // Right
    mesh.addFace(1, 5, 6);
    mesh.addFace(1, 6, 2);

    mesh.computeFaceNormals();
    mesh.computeVertexNormals();

    assert(mesh.vertexCount() == 8);
    assert(mesh.faceCount() == 12);

    std::cout << "PASSED\n";
}

void testQuadric() {
    std::cout << "Test: QEM quadric calculation... ";

    Vector3d normal(0, 0, 1);
    Vector3d point(0, 0, 0);

    Matrix4d Q = QuadricCalculator::computeFaceQuadric(normal, point);

    // Point on plane should have zero error
    double err = QuadricCalculator::computeError(Q, point);
    assert(err < 1e-10);

    // Point off plane should have non-zero error
    Vector3d offPlane(0, 0, 1);
    double errOff = QuadricCalculator::computeError(Q, offPlane);
    assert(errOff > 0.9);  // Should be approximately 1

    std::cout << "PASSED\n";
}

int main() {
    std::cout << "=== Shape-Preserving Simplification Tests ===\n\n";

    testMeshBasic();
    testMeshCube();
    testQuadric();

    std::cout << "\nAll tests passed!\n";
    return 0;
}
