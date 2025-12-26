#include "sps/io/mesh_io.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

namespace sps {

std::string MeshIO::getExtension(const std::string& filename) {
    auto pos = filename.rfind('.');
    if (pos == std::string::npos) return "";
    std::string ext = filename.substr(pos + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext;
}

bool MeshIO::load(const std::string& filename, Mesh& mesh) {
    std::string ext = getExtension(filename);
    if (ext == "obj") return loadOBJ(filename, mesh);
    if (ext == "ply") return loadPLY(filename, mesh);
    std::cerr << "Unknown format: " << ext << std::endl;
    return false;
}

bool MeshIO::save(const std::string& filename, const Mesh& mesh) {
    std::string ext = getExtension(filename);
    if (ext == "obj") return saveOBJ(filename, mesh);
    if (ext == "ply") return savePLY(filename, mesh);
    std::cerr << "Unknown format: " << ext << std::endl;
    return false;
}

bool MeshIO::loadOBJ(const std::string& filename, Mesh& mesh) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open: " << filename << std::endl;
        return false;
    }

    mesh.clear();
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            double x, y, z;
            iss >> x >> y >> z;
            mesh.addVertex(Vector3d(x, y, z));
        }
        else if (prefix == "vt") {
            double u, v;
            iss >> u >> v;
            mesh.texCoords.push_back(Vector2d(u, v));
        }
        else if (prefix == "vn") {
            double x, y, z;
            iss >> x >> y >> z;
            mesh.normals.push_back(Vector3d(x, y, z).normalized());
        }
        else if (prefix == "f") {
            std::vector<Index> vIdx, vtIdx, vnIdx;
            std::string token;
            while (iss >> token) {
                Index v = 0, vt = 0, vn = 0;
                size_t p1 = token.find('/');
                if (p1 == std::string::npos) {
                    v = std::stoi(token);
                } else {
                    v = std::stoi(token.substr(0, p1));
                    size_t p2 = token.find('/', p1 + 1);
                    if (p2 == std::string::npos) {
                        if (p1 + 1 < token.size())
                            vt = std::stoi(token.substr(p1 + 1));
                    } else {
                        if (p2 > p1 + 1)
                            vt = std::stoi(token.substr(p1 + 1, p2 - p1 - 1));
                        if (p2 + 1 < token.size())
                            vn = std::stoi(token.substr(p2 + 1));
                    }
                }
                vIdx.push_back(v - 1);  // OBJ is 1-indexed
                vtIdx.push_back(vt - 1);
                vnIdx.push_back(vn - 1);
            }
            // Triangulate if needed
            for (size_t i = 1; i + 1 < vIdx.size(); ++i) {
                mesh.addFace(vIdx[0], vIdx[i], vIdx[i + 1]);
            }
        }
    }

    mesh.computeFaceNormals();
    mesh.computeVertexNormals();
    std::cout << "Loaded: " << filename << " (V=" << mesh.vertexCount()
              << ", F=" << mesh.faceCount() << ")" << std::endl;
    return true;
}

bool MeshIO::saveOBJ(const std::string& filename, const Mesh& mesh) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot write: " << filename << std::endl;
        return false;
    }

    file << "# Shape-Preserving Simplification Output\n";
    file << "# Vertices: " << mesh.vertexCount() << "\n";
    file << "# Faces: " << mesh.faceCount() << "\n\n";

    // Write vertices
    for (const auto& v : mesh.vertices) {
        if (!v.removed) {
            file << "v " << v.position.x() << " "
                 << v.position.y() << " " << v.position.z() << "\n";
        }
    }

    // Write faces (need to remap indices if there are removed vertices)
    std::vector<Index> indexMap(mesh.vertices.size(), -1);
    Index newIdx = 0;
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        if (!mesh.vertices[i].removed) {
            indexMap[i] = newIdx++;
        }
    }

    for (const auto& f : mesh.faces) {
        if (!f.removed) {
            file << "f " << (indexMap[f.vertices[0]] + 1) << " "
                 << (indexMap[f.vertices[1]] + 1) << " "
                 << (indexMap[f.vertices[2]] + 1) << "\n";
        }
    }

    std::cout << "Saved: " << filename << std::endl;
    return true;
}

bool MeshIO::loadPLY(const std::string&, Mesh&) {
    // TODO: implement PLY loading
    std::cerr << "PLY loading not implemented yet" << std::endl;
    return false;
}

bool MeshIO::savePLY(const std::string&, const Mesh&) {
    // TODO: implement PLY saving
    std::cerr << "PLY saving not implemented yet" << std::endl;
    return false;
}

} // namespace sps
