/*
 * MRPT GridMap Loader
 *
 * This class loads occupancy grids saved by MRPT (Mobile Robot Programming Toolkit)
 * in .gridmap format and converts them to the Gridder component's internal format.
 *
 * MRPT format:
 * - Binary serialization of COccupancyGridMap2D objects
 * - Stores occupancy probabilities as log-odds
 * - Includes metadata: resolution, origin, dimensions
 *
 * Conversion strategy:
 * - Read MRPT binary format
 * - Extract cell occupancy values (0-100 range or -infinity to log-odds)
 * - Convert to Gridder cell format
 * - Support both dense and sparse grid formats
 */

#ifndef MRPT_MAP_LOADER_H
#define MRPT_MAP_LOADER_H

#include <string>
#include <vector>
#include <cstdint>
#include <Eigen/Dense>
#include <optional>
#include <iostream>
#include <fstream>

class MRPTMapLoader
{
public:
    // MRPT cell data (what we read from the file)
    struct MRPTCell
    {
        int32_t x, y;           // Grid coordinates (millimeters)
        float occupancy;        // Occupancy probability (0.0 = free, 1.0 = occupied, -1.0 = unknown)
    };

    // Metadata from MRPT gridmap
    struct MapMetadata
    {
        float resolution = 100.f;           // mm per cell (typical: 100mm = 0.1m)
        int32_t origin_x = 0;               // mm
        int32_t origin_y = 0;               // mm
        int32_t size_x = 0;                 // cells
        int32_t size_y = 0;                 // cells
        std::string name;                   // Map name
        std::string timestamp;              // Map creation timestamp
    };

    // Result structure for loading
    struct LoadResult
    {
        bool success = false;
        std::string error_msg;
        std::string filepath;
        MapMetadata metadata;
        std::vector<MRPTCell> cells;
        int32_t num_occupied = 0;
        int32_t num_free = 0;
        int32_t num_unknown = 0;
    };

    /**
     * @brief Load an MRPT gridmap file (.gridmap extension)
     *
     * @param filepath Full path to the .gridmap file
     * @return LoadResult with parsed data and metadata or error information
     */
    static LoadResult load_gridmap(const std::string &filepath);

    /**
     * @brief Load an MRPT gridmap from a memory buffer
     *
     * @param buffer Raw file data
     * @param size Size of buffer in bytes
     * @return LoadResult with parsed data
     */
    static LoadResult load_gridmap_from_buffer(const uint8_t *buffer, size_t size);

    /**
     * @brief Convert MRPT occupancy value to normalized cost (0-255 or 0.0-100.0)
     *
     * @param mrpt_occupancy Occupancy from MRPT (typically 0.0-1.0 or 0-100)
     * @return Normalized cost value
     */
    static float convert_occupancy_to_cost(float mrpt_occupancy);

    /**
     * @brief Convert MRPT occupancy to binary free/occupied state
     *
     * Uses thresholding with hysteresis for robustness
     *
     * @param mrpt_occupancy Occupancy probability
     * @param threshold Threshold for occupation (default 0.5)
     * @return true if occupied, false if free/unknown
     */
    static bool is_occupied(float mrpt_occupancy, float threshold = 0.5f);

    /**
     * @brief Convert MRPT coordinate to world coordinates
     *
     * @param grid_x X index in grid
     * @param grid_y Y index in grid
     * @param metadata Map metadata with origin and resolution
     * @return World coordinates in mm
     */
    static Eigen::Vector2f grid_to_world(int32_t grid_x, int32_t grid_y, const MapMetadata &metadata);

    /**
     * @brief Create a simple text report of the loaded map
     */
    static std::string create_summary_report(const LoadResult &result);

private:
    // Binary format parsing helpers
    static bool try_read_mrpt_native_format(std::ifstream &file, LoadResult &result);
    static bool try_read_mrpt_legacy_format(std::ifstream &file, LoadResult &result);
    static bool try_read_mrpt_yaml_format(std::ifstream &file, LoadResult &result);

    // Utility functions
    static uint32_t read_uint32(std::ifstream &file);
    static float read_float(std::ifstream &file);
    static std::string read_string(std::ifstream &file);
    static bool skip_bytes(std::ifstream &file, size_t count);
};

#endif // MRPT_MAP_LOADER_H
