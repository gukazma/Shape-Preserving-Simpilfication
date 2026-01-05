#pragma once

#include "sps/core/mesh.h"
#include <string>

// CGAL I/O headers
#include <CGAL/IO/OBJ.h>
#include <CGAL/IO/PLY.h>
#include <CGAL/IO/OFF.h>
#include <CGAL/IO/STL.h>

namespace sps {

/**
 * Mesh I/O utilities using CGAL
 */
class MeshIO {
public:
    /**
     * Load mesh from file (auto-detect format by extension)
     * Supported formats: OBJ, PLY, OFF, STL
     */
    static bool load(const std::string& filename, Mesh& mesh);

    /**
     * Save mesh to file (auto-detect format by extension)
     * Supported formats: OBJ, PLY, OFF, STL
     */
    static bool save(const std::string& filename, const Mesh& mesh);

    /**
     * Load OBJ file
     */
    static bool loadOBJ(const std::string& filename, Mesh& mesh);

    /**
     * Save OBJ file
     */
    static bool saveOBJ(const std::string& filename, const Mesh& mesh);

    /**
     * Load PLY file
     */
    static bool loadPLY(const std::string& filename, Mesh& mesh);

    /**
     * Save PLY file
     */
    static bool savePLY(const std::string& filename, const Mesh& mesh);

    /**
     * Load OFF file
     */
    static bool loadOFF(const std::string& filename, Mesh& mesh);

    /**
     * Save OFF file
     */
    static bool saveOFF(const std::string& filename, const Mesh& mesh);

    /**
     * Load STL file
     */
    static bool loadSTL(const std::string& filename, Mesh& mesh);

    /**
     * Save STL file
     */
    static bool saveSTL(const std::string& filename, const Mesh& mesh);

private:
    // Get file extension (lowercase)
    static std::string getExtension(const std::string& filename);

    // Post-load processing (compute normals, bounding box, etc.)
    static void postLoad(Mesh& mesh, const std::string& filename);
};

} // namespace sps
