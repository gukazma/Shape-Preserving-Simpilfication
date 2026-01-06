#include "sps/io/mesh_io.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <unordered_map>
#include <filesystem>

// CGAL Polygon Mesh Processing for mesh repair
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/merge_border_vertices.h>
#include <CGAL/Polygon_mesh_processing/border.h>

namespace PMP = CGAL::Polygon_mesh_processing;
namespace fs = std::filesystem;

namespace sps {

std::string MeshIO::getExtension(const std::string& filename) {
    auto pos = filename.rfind('.');
    if (pos == std::string::npos) return "";
    std::string ext = filename.substr(pos + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext;
}

void MeshIO::postLoad(Mesh& mesh, const std::string& filename) {
    // Extract name from filename
    auto lastSlash = filename.find_last_of("/\\");
    mesh.name = (lastSlash != std::string::npos)
        ? filename.substr(lastSlash + 1)
        : filename;

    CGALMesh& sm = mesh.cgal_mesh;

    // Step 1: Count border edges before repair
    std::vector<CGALMesh::Halfedge_index> border_cycles;
    PMP::extract_boundary_cycles(sm, std::back_inserter(border_cycles));
    size_t initialBorderCycles = border_cycles.size();

    // Step 2: Merge border vertices that are geometrically close
    // This handles cases where vertices have slightly different coordinates
    size_t verticesBefore = sm.number_of_vertices();
    PMP::merge_duplicated_vertices_in_boundary_cycles(sm);
    size_t mergedVertices = verticesBefore - sm.number_of_vertices();

    // Step 3: Stitch borders - connect matching border halfedges
    size_t stitchedPairs = PMP::stitch_borders(sm);

    // Step 4: Remove any isolated vertices created during repair
    size_t removedIsolated = PMP::remove_isolated_vertices(sm);

    // Report repair statistics
    if (mergedVertices > 0 || stitchedPairs > 0 || removedIsolated > 0) {
        std::cout << "Mesh repair:\n";
        if (mergedVertices > 0)
            std::cout << "  - Merged " << mergedVertices << " duplicated border vertices\n";
        if (stitchedPairs > 0)
            std::cout << "  - Stitched " << stitchedPairs << " border halfedge pairs\n";
        if (removedIsolated > 0)
            std::cout << "  - Removed " << removedIsolated << " isolated vertices\n";
    }

    // Count border edges after repair
    border_cycles.clear();
    PMP::extract_boundary_cycles(sm, std::back_inserter(border_cycles));

    if (!border_cycles.empty()) {
        std::cout << "Warning: Mesh has " << border_cycles.size() << " boundary cycles (not watertight)\n";
    }

    // Initialize property maps
    mesh.initPropertyMaps();

    // Compute geometry
    mesh.computeFaceNormals();
    mesh.computeBoundingBox();

    std::cout << "Loaded: " << filename
              << " (V=" << mesh.vertexCount()
              << ", F=" << mesh.faceCount()
              << ", E=" << mesh.edgeCount() << ")\n";
}

bool MeshIO::load(const std::string& filename, Mesh& mesh) {
    std::string ext = getExtension(filename);

    if (ext == "obj") return loadOBJ(filename, mesh);
    if (ext == "ply") return loadPLY(filename, mesh);
    if (ext == "off") return loadOFF(filename, mesh);
    if (ext == "stl") return loadSTL(filename, mesh);

    std::cerr << "Unknown format: " << ext << std::endl;
    return false;
}

bool MeshIO::save(const std::string& filename, const Mesh& mesh) {
    std::string ext = getExtension(filename);

    if (ext == "obj") return saveOBJ(filename, mesh);
    if (ext == "ply") return savePLY(filename, mesh);
    if (ext == "off") return saveOFF(filename, mesh);
    if (ext == "stl") return saveSTL(filename, mesh);

    std::cerr << "Unknown format: " << ext << std::endl;
    return false;
}

bool MeshIO::loadOBJ(const std::string& filename, Mesh& mesh) {
    mesh.clear();

    std::ifstream input(filename);
    if (!input.is_open()) {
        std::cerr << "Cannot open: " << filename << std::endl;
        return false;
    }

    // First pass: collect vertices, texcoords, and face data
    std::vector<Point_3> vertices;
    std::vector<Point_2> texcoords;
    std::vector<std::array<int, 3>> faceVertexIndices;
    std::vector<std::array<int, 3>> faceTexcoordIndices;
    std::vector<int> faceMaterialIndices;  // Material index per face
    std::vector<std::string> materialNames;  // List of material names
    std::map<std::string, int> materialNameToIndex;  // Map name to index
    std::string mtlFile;
    int currentMaterial = 0;

    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            double x, y, z;
            iss >> x >> y >> z;
            vertices.emplace_back(x, y, z);
        }
        else if (prefix == "vt") {
            double u, v;
            iss >> u >> v;
            texcoords.emplace_back(u, v);
        }
        else if (prefix == "usemtl") {
            std::string matName;
            iss >> matName;
            auto it = materialNameToIndex.find(matName);
            if (it == materialNameToIndex.end()) {
                currentMaterial = static_cast<int>(materialNames.size());
                materialNames.push_back(matName);
                materialNameToIndex[matName] = currentMaterial;
            } else {
                currentMaterial = it->second;
            }
        }
        else if (prefix == "f") {
            std::array<int, 3> vIdx = {-1, -1, -1};
            std::array<int, 3> vtIdx = {-1, -1, -1};

            for (int i = 0; i < 3; ++i) {
                std::string token;
                iss >> token;
                if (token.empty()) break;

                // Parse v/vt/vn or v/vt or v//vn or v format
                size_t pos1 = token.find('/');
                if (pos1 == std::string::npos) {
                    vIdx[i] = std::stoi(token) - 1;  // OBJ indices are 1-based
                } else {
                    vIdx[i] = std::stoi(token.substr(0, pos1)) - 1;
                    size_t pos2 = token.find('/', pos1 + 1);
                    if (pos2 != pos1 + 1) {  // Has texture coord
                        std::string vtStr = (pos2 != std::string::npos)
                            ? token.substr(pos1 + 1, pos2 - pos1 - 1)
                            : token.substr(pos1 + 1);
                        if (!vtStr.empty()) {
                            vtIdx[i] = std::stoi(vtStr) - 1;
                        }
                    }
                }
            }

            if (vIdx[0] >= 0 && vIdx[1] >= 0 && vIdx[2] >= 0) {
                faceVertexIndices.push_back(vIdx);
                faceTexcoordIndices.push_back(vtIdx);
                faceMaterialIndices.push_back(currentMaterial);
            }
        }
        else if (prefix == "mtllib") {
            iss >> mtlFile;
        }
    }
    input.close();

    // Build CGAL mesh
    CGALMesh& sm = mesh.cgal_mesh;
    std::vector<vertex_descriptor> vertexHandles;
    vertexHandles.reserve(vertices.size());

    for (const auto& v : vertices) {
        vertexHandles.push_back(sm.add_vertex(v));
    }

    // Check if we have texture coordinates
    bool hasTexcoords = !texcoords.empty();
    for (const auto& vtIdx : faceTexcoordIndices) {
        if (vtIdx[0] < 0 || vtIdx[1] < 0 || vtIdx[2] < 0) {
            hasTexcoords = false;
            break;
        }
    }

    if (hasTexcoords) {
        mesh.initTexcoordMap();
    }

    // Check if we have multiple materials
    bool hasMaterials = materialNames.size() > 1;
    if (hasMaterials) {
        mesh.initMaterialMap();
        mesh.setMaterialNames(materialNames);
    }

    // Add faces and set texture coordinates and materials
    for (size_t i = 0; i < faceVertexIndices.size(); ++i) {
        const auto& vIdx = faceVertexIndices[i];
        face_descriptor f = sm.add_face(vertexHandles[vIdx[0]],
                                         vertexHandles[vIdx[1]],
                                         vertexHandles[vIdx[2]]);

        if (f != CGALMesh::null_face()) {
            // Set texture coordinates
            if (hasTexcoords) {
                const auto& vtIdx = faceTexcoordIndices[i];
                halfedge_descriptor h = sm.halfedge(f);

                // Set texcoords for each halfedge of the face
                for (int j = 0; j < 3; ++j) {
                    if (vtIdx[j] >= 0 && vtIdx[j] < static_cast<int>(texcoords.size())) {
                        mesh.setTexcoord(h, texcoords[vtIdx[j]]);
                    }
                    h = sm.next(h);
                }
            }

            // Set material index
            if (hasMaterials) {
                mesh.setFaceMaterial(f, faceMaterialIndices[i]);
            }
        }
    }

    // Store MTL filename for later use
    if (!mtlFile.empty()) {
        fs::path objPath(filename);
        mesh.mtlFile = (objPath.parent_path() / mtlFile).string();
    }

    if (hasTexcoords) {
        std::cout << "Loaded " << texcoords.size() << " texture coordinates\n";
    }
    if (hasMaterials) {
        std::cout << "Loaded " << materialNames.size() << " materials\n";
    }

    postLoad(mesh, filename);
    return true;
}

bool MeshIO::saveOBJ(const std::string& filename, const Mesh& mesh) {
    std::ofstream output(filename);
    if (!output.is_open()) {
        std::cerr << "Cannot write: " << filename << std::endl;
        return false;
    }

    const CGALMesh& sm = mesh.cgal_mesh;
    fs::path outPath(filename);
    std::string outDir = outPath.parent_path().string();
    std::string baseName = outPath.stem().string();

    // Copy MTL file if it exists and write mtllib reference
    if (!mesh.mtlFile.empty() && fs::exists(mesh.mtlFile)) {
        fs::path srcMtl(mesh.mtlFile);
        std::string newMtlName = baseName + ".mtl";
        fs::path dstMtl = outPath.parent_path() / newMtlName;

        // Check if source and destination are the same (e.g., baked texture MTL)
        bool sameFile = false;
        try {
            sameFile = fs::equivalent(srcMtl, dstMtl);
        } catch (...) {
            sameFile = (srcMtl == dstMtl);
        }

        if (sameFile) {
            // MTL file already in place, just write reference
            output << "mtllib " << newMtlName << "\n";
        } else {
            // Copy MTL file
            try {
                fs::copy_file(srcMtl, dstMtl, fs::copy_options::overwrite_existing);

                // Copy referenced texture files
                std::ifstream mtlIn(mesh.mtlFile);
                std::string line;
                while (std::getline(mtlIn, line)) {
                    if (line.find("map_Kd") != std::string::npos ||
                        line.find("map_Ka") != std::string::npos ||
                        line.find("map_Ks") != std::string::npos) {
                        std::istringstream iss(line);
                        std::string prefix, texFile;
                        iss >> prefix >> texFile;
                        if (!texFile.empty()) {
                            fs::path srcTex = srcMtl.parent_path() / texFile;
                            fs::path dstTex = outPath.parent_path() / texFile;
                            if (fs::exists(srcTex) && srcTex != dstTex) {
                                fs::copy_file(srcTex, dstTex, fs::copy_options::overwrite_existing);
                            }
                        }
                    }
                }
                output << "mtllib " << newMtlName << "\n";
            } catch (const std::exception& e) {
                std::cerr << "Warning: Could not copy MTL file: " << e.what() << "\n";
            }
        }
    }

    // Write header comment
    output << "# Shape-Preserving Simplification Output\n";
    output << "# Vertices: " << mesh.vertexCount() << "\n";
    output << "# Faces: " << mesh.faceCount() << "\n\n";

    // Build vertex index map
    std::unordered_map<vertex_descriptor, int> vertexIndex;
    int vIdx = 1;  // OBJ indices are 1-based

    // Write vertices
    for (vertex_descriptor v : sm.vertices()) {
        const Point_3& p = sm.point(v);
        output << "v " << CGAL::to_double(p.x()) << " "
               << CGAL::to_double(p.y()) << " "
               << CGAL::to_double(p.z()) << "\n";
        vertexIndex[v] = vIdx++;
    }

    // Write texture coordinates and build texcoord index map
    std::unordered_map<halfedge_descriptor, int> texcoordIndex;
    int vtIdx = 1;

    if (mesh.hasTexcoords()) {
        output << "\n# Texture coordinates\n";

        // Collect unique texcoords (with small epsilon for comparison)
        std::vector<Point_2> uniqueTexcoords;
        std::unordered_map<std::string, int> texcoordMap;

        for (face_descriptor f : sm.faces()) {
            halfedge_descriptor h = sm.halfedge(f);
            for (int i = 0; i < 3; ++i) {
                Point_2 uv = mesh.texcoord(h);
                // Create a key with limited precision
                std::ostringstream key;
                key << std::fixed << std::setprecision(6)
                    << CGAL::to_double(uv.x()) << "_" << CGAL::to_double(uv.y());
                std::string keyStr = key.str();

                auto it = texcoordMap.find(keyStr);
                if (it == texcoordMap.end()) {
                    uniqueTexcoords.push_back(uv);
                    texcoordMap[keyStr] = vtIdx;
                    texcoordIndex[h] = vtIdx;
                    vtIdx++;
                } else {
                    texcoordIndex[h] = it->second;
                }
                h = sm.next(h);
            }
        }

        // Write unique texture coordinates
        for (const auto& uv : uniqueTexcoords) {
            output << "vt " << CGAL::to_double(uv.x()) << " "
                   << CGAL::to_double(uv.y()) << "\n";
        }
    }

    // Write faces (grouped by material if multi-material)
    output << "\n# Faces\n";

    if (mesh.hasMaterials()) {
        // Group faces by material
        const auto& materialNames = mesh.getMaterialNames();
        int numMaterials = static_cast<int>(materialNames.size());

        // Create face lists per material
        std::vector<std::vector<face_descriptor>> facesByMaterial(numMaterials);
        for (face_descriptor f : sm.faces()) {
            int matIdx = mesh.faceMaterial(f);
            if (matIdx >= 0 && matIdx < numMaterials) {
                facesByMaterial[matIdx].push_back(f);
            } else {
                facesByMaterial[0].push_back(f);  // Default to first material
            }
        }

        // Write faces grouped by material
        for (int matIdx = 0; matIdx < numMaterials; ++matIdx) {
            if (facesByMaterial[matIdx].empty()) continue;

            output << "usemtl " << materialNames[matIdx] << "\n";

            for (face_descriptor f : facesByMaterial[matIdx]) {
                halfedge_descriptor h = sm.halfedge(f);
                output << "f";
                for (int i = 0; i < 3; ++i) {
                    vertex_descriptor v = sm.target(h);
                    if (mesh.hasTexcoords()) {
                        output << " " << vertexIndex[v] << "/" << texcoordIndex[h];
                    } else {
                        output << " " << vertexIndex[v];
                    }
                    h = sm.next(h);
                }
                output << "\n";
            }
        }
    } else if (mesh.hasTexcoords()) {
        for (face_descriptor f : sm.faces()) {
            halfedge_descriptor h = sm.halfedge(f);
            output << "f";
            for (int i = 0; i < 3; ++i) {
                vertex_descriptor v = sm.target(h);
                output << " " << vertexIndex[v] << "/" << texcoordIndex[h];
                h = sm.next(h);
            }
            output << "\n";
        }
    } else {
        for (face_descriptor f : sm.faces()) {
            halfedge_descriptor h = sm.halfedge(f);
            output << "f";
            for (int i = 0; i < 3; ++i) {
                vertex_descriptor v = sm.target(h);
                output << " " << vertexIndex[v];
                h = sm.next(h);
            }
            output << "\n";
        }
    }

    std::cout << "Saved: " << filename
              << " (V=" << mesh.vertexCount()
              << ", F=" << mesh.faceCount();
    if (mesh.hasTexcoords()) {
        std::cout << ", VT=" << (vtIdx - 1);
    }
    std::cout << ")\n";
    return true;
}

bool MeshIO::loadPLY(const std::string& filename, Mesh& mesh) {
    mesh.clear();

    std::ifstream input(filename, std::ios::binary);
    if (!input.is_open()) {
        std::cerr << "Cannot open: " << filename << std::endl;
        return false;
    }

    if (!CGAL::IO::read_PLY(input, mesh.cgal_mesh)) {
        std::cerr << "Failed to read PLY: " << filename << std::endl;
        return false;
    }

    postLoad(mesh, filename);
    return true;
}

bool MeshIO::savePLY(const std::string& filename, const Mesh& mesh) {
    std::ofstream output(filename, std::ios::binary);
    if (!output.is_open()) {
        std::cerr << "Cannot write: " << filename << std::endl;
        return false;
    }

    if (!CGAL::IO::write_PLY(output, mesh.cgal_mesh)) {
        std::cerr << "Failed to write PLY: " << filename << std::endl;
        return false;
    }

    std::cout << "Saved: " << filename
              << " (V=" << mesh.vertexCount()
              << ", F=" << mesh.faceCount() << ")\n";
    return true;
}

bool MeshIO::loadOFF(const std::string& filename, Mesh& mesh) {
    mesh.clear();

    std::ifstream input(filename);
    if (!input.is_open()) {
        std::cerr << "Cannot open: " << filename << std::endl;
        return false;
    }

    if (!CGAL::IO::read_OFF(input, mesh.cgal_mesh)) {
        std::cerr << "Failed to read OFF: " << filename << std::endl;
        return false;
    }

    postLoad(mesh, filename);
    return true;
}

bool MeshIO::saveOFF(const std::string& filename, const Mesh& mesh) {
    std::ofstream output(filename);
    if (!output.is_open()) {
        std::cerr << "Cannot write: " << filename << std::endl;
        return false;
    }

    if (!CGAL::IO::write_OFF(output, mesh.cgal_mesh)) {
        std::cerr << "Failed to write OFF: " << filename << std::endl;
        return false;
    }

    std::cout << "Saved: " << filename
              << " (V=" << mesh.vertexCount()
              << ", F=" << mesh.faceCount() << ")\n";
    return true;
}

bool MeshIO::loadSTL(const std::string& filename, Mesh& mesh) {
    mesh.clear();

    std::ifstream input(filename, std::ios::binary);
    if (!input.is_open()) {
        std::cerr << "Cannot open: " << filename << std::endl;
        return false;
    }

    if (!CGAL::IO::read_STL(input, mesh.cgal_mesh)) {
        std::cerr << "Failed to read STL: " << filename << std::endl;
        return false;
    }

    postLoad(mesh, filename);
    return true;
}

bool MeshIO::saveSTL(const std::string& filename, const Mesh& mesh) {
    std::ofstream output(filename, std::ios::binary);
    if (!output.is_open()) {
        std::cerr << "Cannot write: " << filename << std::endl;
        return false;
    }

    if (!CGAL::IO::write_STL(output, mesh.cgal_mesh)) {
        std::cerr << "Failed to write STL: " << filename << std::endl;
        return false;
    }

    std::cout << "Saved: " << filename
              << " (V=" << mesh.vertexCount()
              << ", F=" << mesh.faceCount() << ")\n";
    return true;
}

} // namespace sps
