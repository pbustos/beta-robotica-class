/*
 * Example: Loading MRPT Maps in Gridder Component
 *
 * This file demonstrates how to use the MRPT map loader
 * in the Gridder component for various use cases.
 */

#include "mrpt_map_loader.h"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

// ============================================================================
// Example 1: Simple map loading with error handling
// ============================================================================
void example_simple_load(const std::string &map_path)
{
    std::cout << "Example 1: Simple Map Loading\n";
    std::cout << "================================\n\n";

    auto result = MRPTMapLoader::load_gridmap(map_path);

    if (!result.success)
    {
        std::cerr << "Failed to load map: " << result.error_msg << "\n";
        return;
    }

    // Print basic information
    std::cout << "Map loaded successfully!\n";
    std::cout << "Resolution: " << result.metadata.resolution << " mm/cell\n";
    std::cout << "Grid size: " << result.metadata.size_x << " x "
              << result.metadata.size_y << " cells\n";
    std::cout << "Total cells: " << result.cells.size() << "\n";
    std::cout << "  - Occupied: " << result.num_occupied << "\n";
    std::cout << "  - Free: " << result.num_free << "\n";
    std::cout << "  - Unknown: " << result.num_unknown << "\n\n";
}

// ============================================================================
// Example 2: Validate map before using
// ============================================================================
void example_validate_map(const std::string &map_path)
{
    std::cout << "Example 2: Map Validation\n";
    std::cout << "==========================\n\n";

    auto result = MRPTMapLoader::load_gridmap(map_path);

    if (!result.success)
    {
        std::cerr << "Cannot validate: " << result.error_msg << "\n";
        return;
    }

    // Validation checks
    bool valid = true;

    if (result.cells.empty())
    {
        std::cerr << "WARNING: No cells loaded!\n";
        valid = false;
    }

    if (result.metadata.resolution <= 0 || result.metadata.resolution > 1000)
    {
        std::cerr << "WARNING: Suspicious resolution: " << result.metadata.resolution << "\n";
        valid = false;
    }

    if (result.num_occupied == 0)
    {
        std::cerr << "WARNING: No obstacles found in map!\n";
        valid = false;
    }

    const int total = result.num_occupied + result.num_free + result.num_unknown;
    if (total > 0)
    {
        float occupied_percentage = (float)result.num_occupied / total * 100.0f;
        std::cout << "Occupancy: " << occupied_percentage << "%\n";

        if (occupied_percentage > 95)
            std::cout << "WARNING: Map is very densely occupied!\n";
        if (occupied_percentage < 1)
            std::cout << "WARNING: Map has very few obstacles!\n";
    }

    std::cout << (valid ? "✓ Map validation PASSED\n" : "✗ Map has issues\n");
    std::cout << "\n";
}

// ============================================================================
// Example 3: Find all MRPT maps in a directory
// ============================================================================
void example_find_maps(const std::string &directory)
{
    std::cout << "Example 3: Find MRPT Maps in Directory\n";
    std::cout << "======================================\n\n";

    if (!fs::exists(directory))
    {
        std::cerr << "Directory not found: " << directory << "\n";
        return;
    }

    int count = 0;
    for (const auto &entry : fs::recursive_directory_iterator(directory))
    {
        if (entry.path().extension() == ".gridmap")
        {
            std::cout << "Found: " << entry.path().string() << "\n";

            auto result = MRPTMapLoader::load_gridmap(entry.path().string());
            if (result.success)
            {
                std::cout << "  ✓ " << result.cells.size() << " cells loaded\n";
                count++;
            }
            else
            {
                std::cout << "  ✗ " << result.error_msg << "\n";
            }
        }
    }

    std::cout << "\nFound " << count << " valid maps\n\n";
}

// ============================================================================
// Example 4: Compare occupancy distributions between maps
// ============================================================================
void example_compare_maps(const std::vector<std::string> &map_paths)
{
    std::cout << "Example 4: Compare Maps\n";
    std::cout << "========================\n\n";

    for (const auto &path : map_paths)
    {
        auto result = MRPTMapLoader::load_gridmap(path);

        if (!result.success)
        {
            std::cout << path << " - FAILED\n";
            continue;
        }

        const int total = result.num_occupied + result.num_free + result.num_unknown;
        float occ_pct = (total > 0) ? (float)result.num_occupied / total * 100.0f : 0;

        std::cout << fs::path(path).filename().string() << "\n"
                  << "  Cells: " << result.cells.size()
                  << " | Occupancy: " << occ_pct << "%"
                  << " | Resolution: " << result.metadata.resolution << " mm\n";
    }
    std::cout << "\n";
}

// ============================================================================
// Example 5: Calculate statistics for map
// ============================================================================
void example_map_statistics(const std::string &map_path)
{
    std::cout << "Example 5: Map Statistics\n";
    std::cout << "==========================\n\n";

    auto result = MRPTMapLoader::load_gridmap(map_path);

    if (!result.success)
    {
        std::cerr << "Failed: " << result.error_msg << "\n";
        return;
    }

    // Calculate spatial extent
    int32_t min_x = INT32_MAX, max_x = INT32_MIN;
    int32_t min_y = INT32_MAX, max_y = INT32_MIN;

    for (const auto &cell : result.cells)
    {
        min_x = std::min(min_x, cell.x);
        max_x = std::max(max_x, cell.x);
        min_y = std::min(min_y, cell.y);
        max_y = std::max(max_y, cell.y);
    }

    int32_t width = max_x - min_x;
    int32_t height = max_y - min_y;

    std::cout << "Spatial Statistics:\n";
    std::cout << "  X range: [" << min_x << ", " << max_x << "] ("
              << width << " mm)\n";
    std::cout << "  Y range: [" << min_y << ", " << max_y << "] ("
              << height << " mm)\n";
    std::cout << "  Area: " << (width * height) / 1e6 << " m²\n\n";

    std::cout << "Occupancy Statistics:\n";
    const int total = result.num_occupied + result.num_free + result.num_unknown;
    std::cout << "  Occupied: " << result.num_occupied
              << " (" << (float)result.num_occupied/total*100 << "%)\n";
    std::cout << "  Free: " << result.num_free
              << " (" << (float)result.num_free/total*100 << "%)\n";
    std::cout << "  Unknown: " << result.num_unknown
              << " (" << (float)result.num_unknown/total*100 << "%)\n\n";
}

// ============================================================================
// Example 6: Occupancy conversion reference
// ============================================================================
void example_occupancy_conversion()
{
    std::cout << "Example 6: Occupancy Conversion\n";
    std::cout << "================================\n\n";

    std::cout << "MRPT Occupancy -> Gridder Cost conversion:\n\n";

    float test_values[] = {0.0f, 0.1f, 0.25f, 0.5f, 0.75f, 0.9f, 1.0f};

    for (float occ : test_values)
    {
        float cost = MRPTMapLoader::convert_occupancy_to_cost(occ);
        bool occupied = MRPTMapLoader::is_occupied(occ);

        std::cout << "Occupancy: " << occ
                  << " -> Cost: " << cost
                  << " -> " << (occupied ? "OCCUPIED" : "FREE/UNKNOWN") << "\n";
    }

    std::cout << "\n";
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char **argv)
{
    std::cout << "\n╔════════════════════════════════════════╗\n";
    std::cout << "║  MRPT Map Loader - Examples & Tests    ║\n";
    std::cout << "╚════════════════════════════════════════╝\n\n";

    // Demonstration of conversion
    example_occupancy_conversion();

    // Check if example map is provided
    if (argc > 1)
    {
        std::string map_file = argv[1];

        example_simple_load(map_file);
        example_validate_map(map_file);
        example_map_statistics(map_file);

        // Optional: compare multiple maps
        if (argc > 2)
        {
            std::vector<std::string> maps;
            for (int i = 1; i < argc; ++i)
                maps.push_back(argv[i]);
            example_compare_maps(maps);
        }
    }
    else
    {
        std::cout << "Usage: mrpt_loader_examples <map.gridmap> [map2.gridmap ...]\n\n";
        std::cout << "Examples:\n";
        std::cout << "  ./mrpt_loader_examples my_map.gridmap\n";
        std::cout << "  ./mrpt_loader_examples map1.gridmap map2.gridmap map3.gridmap\n\n";
    }

    return 0;
}
