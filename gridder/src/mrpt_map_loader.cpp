#include "mrpt_map_loader.h"
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cstring>

// Check if file is zstd compressed (magic bytes: 0x28 0xB5 0x2F 0xFD)
static bool is_zstd_compressed(std::ifstream &file)
{
    char magic[4];
    file.read(magic, 4);
    file.seekg(0);
    return (magic[0] == 0x28 && magic[1] == (char)0xB5 &&
            magic[2] == 0x2F && magic[3] == (char)0xFD);
}

// Decompress zstd file using system command (requires zstd installed)
static std::string decompress_zstd_to_temp(const std::string &filepath)
{
    std::string temp_file = "/tmp/mrpt_map_decompressed_" +
                            std::to_string(std::hash<std::string>{}(filepath)) + ".bin";
    std::string cmd = "zstd -d -f -q \"" + filepath + "\" -o \"" + temp_file + "\" 2>/dev/null";
    int result = system(cmd.c_str());
    if (result != 0)
        return "";
    return temp_file;
}

MRPTMapLoader::LoadResult MRPTMapLoader::load_gridmap(const std::string &filepath)
{
    LoadResult result;
    std::string file_to_read = filepath;
    std::string temp_file;

    // Check if file exists
    std::ifstream check_file(filepath, std::ios::binary);
    if (!check_file.is_open())
    {
        result.success = false;
        result.error_msg = "Cannot open file: " + filepath;
        return result;
    }

    // Check for zstd compression
    if (is_zstd_compressed(check_file))
    {
        check_file.close();
        std::cout << "[MRPT] Detected zstd compression, decompressing...\n";
        temp_file = decompress_zstd_to_temp(filepath);
        if (temp_file.empty())
        {
            result.success = false;
            result.error_msg = "Failed to decompress zstd file (is zstd installed?)";
            return result;
        }
        file_to_read = temp_file;
    }
    else
    {
        check_file.close();
    }

    // Open the (possibly decompressed) file
    std::ifstream file(file_to_read, std::ios::binary);
    if (!file.is_open())
    {
        result.success = false;
        result.error_msg = "Cannot open file: " + file_to_read;
        return result;
    }

    // Try MRPT native format first (COccupancyGridMap2D)
    if (!try_read_mrpt_native_format(file, result))
    {
        file.clear();
        file.seekg(0);
        // Try legacy format
        if (!try_read_mrpt_legacy_format(file, result))
        {
            file.clear();
            file.seekg(0);
            // Try YAML format
            if (!try_read_mrpt_yaml_format(file, result))
            {
                result.success = false;
                result.error_msg = "Unknown file format";
            }
        }
    }

    file.close();

    // Clean up temp file
    if (!temp_file.empty())
        std::remove(temp_file.c_str());

    if (result.success)
        std::cout << "[MRPT] Loaded: " << filepath << " - " << result.cells.size() << " cells\n";
    return result;
}

bool MRPTMapLoader::try_read_mrpt_native_format(std::ifstream &file, LoadResult &result)
{
    try
    {
        // Read first byte (class name length indicator)
        uint8_t first_byte;
        file.read(reinterpret_cast<char*>(&first_byte), 1);

        // Check for MRPT class signature (0x9f = 159, but actual name is 31 chars)
        // The format uses variable-length encoding
        size_t name_len = 31;  // "mrpt::maps::COccupancyGridMap2D"

        char class_name[64];
        file.read(class_name, name_len);
        class_name[name_len] = '\0';

        // Verify class name
        if (std::string(class_name) != "mrpt::maps::COccupancyGridMap2D")
            return false;

        std::cout << "[MRPT] Found native format: " << class_name << "\n";

        // Read version (2 bytes)
        uint8_t version_major, version_minor;
        file.read(reinterpret_cast<char*>(&version_major), 1);
        file.read(reinterpret_cast<char*>(&version_minor), 1);
        std::cout << "[MRPT] Version: " << (int)version_major << "." << (int)version_minor << "\n";

        // Read grid dimensions
        uint32_t width, height;
        file.read(reinterpret_cast<char*>(&width), sizeof(uint32_t));
        file.read(reinterpret_cast<char*>(&height), sizeof(uint32_t));
        std::cout << "[MRPT] Grid size: " << width << " x " << height << "\n";

        // Read bounds (floats in meters)
        float x_min, x_max, y_min, y_max, resolution;
        file.read(reinterpret_cast<char*>(&x_min), sizeof(float));
        file.read(reinterpret_cast<char*>(&x_max), sizeof(float));
        file.read(reinterpret_cast<char*>(&y_min), sizeof(float));
        file.read(reinterpret_cast<char*>(&y_max), sizeof(float));
        file.read(reinterpret_cast<char*>(&resolution), sizeof(float));

        std::cout << "[MRPT] Bounds: X[" << x_min << ", " << x_max << "] Y[" << y_min << ", " << y_max << "]\n";
        std::cout << "[MRPT] Resolution: " << resolution << " m (" << resolution * 1000.f << " mm)\n";

        // Store metadata
        result.metadata.size_x = static_cast<int32_t>(width);
        result.metadata.size_y = static_cast<int32_t>(height);
        result.metadata.resolution = resolution * 1000.f;  // Convert to mm
        result.metadata.origin_x = static_cast<int32_t>(x_min * 1000.f);
        result.metadata.origin_y = static_cast<int32_t>(y_min * 1000.f);

        // Skip additional header bytes (padding to data)
        // The header is 62 bytes total, we've read 32 + 2 + 8 + 20 = 62
        // Data starts at position 62

        // Read cell data (each cell is 1 byte: 0=free, 100=occupied, 50=unknown)
        std::vector<uint8_t> cells_raw(width * height);
        file.read(reinterpret_cast<char*>(cells_raw.data()), width * height);

        if (file.gcount() != static_cast<std::streamsize>(width * height))
        {
            std::cerr << "[MRPT] Warning: Read " << file.gcount() << " bytes, expected " << width * height << "\n";
        }

        // Convert to our format - only store occupied cells for sparse representation
        // MRPT format (0-255 range):
        //   0 = unknown/unexplored
        //   ~66 (low values) = free
        //   >127 (high values) = occupied
        result.cells.reserve(width * height / 10);  // Assume ~10% occupied

        for (uint32_t y = 0; y < height; ++y)
        {
            for (uint32_t x = 0; x < width; ++x)
            {
                uint8_t cell_value = cells_raw[y * width + x];

                // MRPT uses 0-255 range:
                // 0 = unknown, low values (1-100) = free, high values (>127) = occupied
                if (cell_value == 0)
                {
                    // Unknown/unexplored - skip (don't add to sparse map)
                    result.num_unknown++;
                }
                else if (cell_value > 127)  // Occupied (high values)
                {
                    MRPTCell cell;
                    cell.x = static_cast<int32_t>(x_min * 1000.f + x * resolution * 1000.f);
                    cell.y = static_cast<int32_t>(y_min * 1000.f + y * resolution * 1000.f);
                    cell.occupancy = static_cast<float>(cell_value) / 255.0f;
                    result.cells.push_back(cell);
                    result.num_occupied++;
                }
                else  // Free (low non-zero values)
                {
                    result.num_free++;
                }
            }
        }

        std::cout << "[MRPT] Parsed: " << result.num_occupied << " occupied, "
                  << result.num_free << " free, " << result.num_unknown << " unknown\n";

        result.success = true;
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[MRPT] Native format error: " << e.what() << "\n";
        return false;
    }
    catch (...)
    {
        return false;
    }
}

bool MRPTMapLoader::try_read_mrpt_legacy_format(std::ifstream &file, LoadResult &result)
{
    try
    {
        uint32_t width = read_uint32(file);
        uint32_t height = read_uint32(file);
        float resolution = read_float(file);
        float origin_x = read_float(file);
        float origin_y = read_float(file);

        if (width == 0 || height == 0 || width > 100000 || height > 100000)
            return false;
        if (resolution <= 0 || resolution > 10000)
            return false;

        result.metadata.size_x = static_cast<int32_t>(width);
        result.metadata.size_y = static_cast<int32_t>(height);
        result.metadata.resolution = resolution;
        result.metadata.origin_x = static_cast<int32_t>(origin_x * 1000.f);
        result.metadata.origin_y = static_cast<int32_t>(origin_y * 1000.f);

        for (uint32_t y = 0; y < height; ++y)
        {
            for (uint32_t x = 0; x < width; ++x)
            {
                float occupancy = read_float(file);
                if (occupancy >= 0.0f)
                {
                    MRPTCell cell;
                    cell.x = static_cast<int32_t>(origin_x * 1000.f + x * resolution);
                    cell.y = static_cast<int32_t>(origin_y * 1000.f + y * resolution);
                    cell.occupancy = occupancy;
                    result.cells.push_back(cell);

                    if (occupancy > 0.5f)
                        result.num_occupied++;
                    else if (occupancy < 0.1f)
                        result.num_free++;
                    else
                        result.num_unknown++;
                }
            }
        }

        result.success = !result.cells.empty();
        return result.success;
    }
    catch (...)
    {
        return false;
    }
}

bool MRPTMapLoader::try_read_mrpt_yaml_format(std::ifstream &file, LoadResult &result)
{
    std::string line;
    int width = 0, height = 0;
    float resolution = 100.f;
    float origin_x = 0.f, origin_y = 0.f;
    bool found_data = false;
    std::vector<std::string> data_lines;

    try
    {
        while (std::getline(file, line))
        {
            line.erase(0, line.find_first_not_of(" \t\r\n"));
            if (!line.empty())
                line.erase(line.find_last_not_of(" \t\r\n") + 1);

            if (line.empty() || line[0] == '#') continue;

            if (line.find("width:") != std::string::npos)
                width = std::stoi(line.substr(line.find(":") + 1));
            else if (line.find("height:") != std::string::npos)
                height = std::stoi(line.substr(line.find(":") + 1));
            else if (line.find("resolution:") != std::string::npos)
                resolution = std::stof(line.substr(line.find(":") + 1));
            else if (line.find("origin_x:") != std::string::npos)
                origin_x = std::stof(line.substr(line.find(":") + 1));
            else if (line.find("origin_y:") != std::string::npos)
                origin_y = std::stof(line.substr(line.find(":") + 1));
            else if (line.find("data:") != std::string::npos)
                found_data = true;
            else if (found_data && !line.empty())
                data_lines.push_back(line);
        }

        if (width == 0 || height == 0 || !found_data)
            return false;

        result.metadata.size_x = width;
        result.metadata.size_y = height;
        result.metadata.resolution = resolution;
        result.metadata.origin_x = static_cast<int32_t>(origin_x * 1000.f);
        result.metadata.origin_y = static_cast<int32_t>(origin_y * 1000.f);

        int cell_idx = 0;
        for (const auto &data_line : data_lines)
        {
            std::istringstream iss(data_line);
            float occupancy;
            while (iss >> occupancy && cell_idx < width * height)
            {
                int y = cell_idx / width;
                int x = cell_idx % width;

                if (occupancy >= 0.0f)
                {
                    MRPTCell cell;
                    cell.x = static_cast<int32_t>(origin_x * 1000.f + x * resolution);
                    cell.y = static_cast<int32_t>(origin_y * 1000.f + y * resolution);
                    cell.occupancy = occupancy;
                    result.cells.push_back(cell);

                    if (occupancy > 0.5f)
                        result.num_occupied++;
                    else if (occupancy < 0.1f)
                        result.num_free++;
                    else
                        result.num_unknown++;
                }
                cell_idx++;
            }
        }

        result.success = !result.cells.empty();
        return result.success;
    }
    catch (...)
    {
        return false;
    }
}

float MRPTMapLoader::convert_occupancy_to_cost(float mrpt_occupancy)
{
    if (mrpt_occupancy < 0.1f)
        return 1.0f;
    else if (mrpt_occupancy > 0.9f)
        return 100.0f;
    else
        return 4.0f;
}

bool MRPTMapLoader::is_occupied(float mrpt_occupancy, float threshold)
{
    return mrpt_occupancy >= threshold;
}

Eigen::Vector2f MRPTMapLoader::grid_to_world(int32_t grid_x, int32_t grid_y, const MapMetadata &metadata)
{
    float world_x = metadata.origin_x + grid_x * metadata.resolution;
    float world_y = metadata.origin_y + grid_y * metadata.resolution;
    return Eigen::Vector2f(world_x, world_y);
}

uint32_t MRPTMapLoader::read_uint32(std::ifstream &file)
{
    uint32_t value;
    file.read(reinterpret_cast<char*>(&value), sizeof(uint32_t));
    return value;
}

float MRPTMapLoader::read_float(std::ifstream &file)
{
    float value;
    file.read(reinterpret_cast<char*>(&value), sizeof(float));
    return value;
}

std::string MRPTMapLoader::read_string(std::ifstream &file)
{
    uint32_t length = read_uint32(file);
    std::string str(length, '\0');
    file.read(&str[0], length);
    return str;
}

bool MRPTMapLoader::skip_bytes(std::ifstream &file, size_t count)
{
    file.seekg(count, std::ios::cur);
    return file.good();
}

std::string MRPTMapLoader::create_summary_report(const LoadResult &result)
{
    std::ostringstream oss;
    oss << "\n===== MRPT Map Loading Report =====\n";
    oss << "Status: " << (result.success ? "SUCCESS" : "FAILED") << "\n";

    if (result.success)
    {
        oss << "Resolution: " << result.metadata.resolution << " mm/cell\n";
        oss << "Grid: " << result.metadata.size_x << " x " << result.metadata.size_y << " cells\n";
        oss << "Cells: " << result.cells.size() << " (occ:" << result.num_occupied
            << " free:" << result.num_free << " unk:" << result.num_unknown << ")\n";
    }
    else
    {
        oss << "Error: " << result.error_msg << "\n";
    }
    oss << "====================================\n";

    return oss.str();
}
