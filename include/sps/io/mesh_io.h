#pragma once

#include "sps/core/mesh.h"
#include <string>

namespace sps {

/**
 * Mesh I/O utilities
 */
class MeshIO {
public:
    /**
     * Load mesh from file (auto-detect format)
     */
    static bool load(const std::string& filename, Mesh& mesh);

    /**
     * Save mesh to file (auto-detect format)
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

private:
    // Get file extension (lowercase)
    static std::string getExtension(const std::string& filename);
};

} // namespace sps
