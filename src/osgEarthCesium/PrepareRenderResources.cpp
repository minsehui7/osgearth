/* osgEarth - Geospatial SDK for OpenSceneGraph
* Copyright 2008-2012 Pelican Mapping
* MIT License
*/
#include "PrepareRenderResources"
#include <CesiumGltfContent/GltfUtilities.h>
#include <CesiumGltf/AccessorView.h>

#include <osg/BlendFunc>
#include <osg/Texture2D>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/NodeVisitor>
#include <osg/PolygonOffset>
#include <osg/Program>
#include <osg/Shader>
#include <osgEarth/ElevationQuery>
#include <osgEarth/ImageUtils>
#include <osgEarth/Lighting>
#include <osgEarth/LineDrawable>
#include <osgEarth/Map>
#include <osgEarth/Notify>
#include <osgEarth/Registry>
#include <osgEarth/SpatialReference>
#include <osgEarth/VirtualProgram>
#include <osg/MatrixTransform>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

using namespace osgEarth::Cesium;

namespace
{
    struct EdgeUse
    {
        unsigned int a = 0u;
        unsigned int b = 0u;
        unsigned int count = 0u;
    };

    struct TileId
    {
        int z = -1;
        int x = -1;
        int y = -1;
    };

    struct TileBoundsRad
    {
        double west = 0.0;
        double south = 0.0;
        double east = 0.0;
        double north = 0.0;
    };

    struct TileLocalFrame
    {
        bool valid = false;
        TileBoundsRad bounds;
        double origin[3]{};
        double east[3]{};
        double north[3]{};
        double up[3]{};
    };

    struct TerrainSampleKey
    {
        std::int64_t lon = 0;
        std::int64_t lat = 0;

        bool operator==(const TerrainSampleKey& rhs) const noexcept
        {
            return lon == rhs.lon && lat == rhs.lat;
        }
    };

    struct TerrainSampleKeyHash
    {
        std::size_t operator()(const TerrainSampleKey& key) const noexcept
        {
            const std::uint64_t lon = static_cast<std::uint64_t>(key.lon);
            const std::uint64_t lat = static_cast<std::uint64_t>(key.lat);
            return static_cast<std::size_t>(lon ^ (lat + 0x9e3779b97f4a7c15ull + (lon << 6u) + (lon >> 2u)));
        }
    };

    struct TerrainSample
    {
        double lonRad = 0.0;
        double latRad = 0.0;
        osg::Vec3d queryPoint;
    };

    const char* kInlineEdgeDrawableName = "h3dt_inline_edges";
    const char* kInlineMeshDrawableName = "h3dt_inline_mesh";

    bool isInlineEdgeDrawable(const osg::Drawable* drawable)
    {
        return drawable && drawable->getName() == kInlineEdgeDrawableName;
    }

    bool isInlineMeshDrawable(const osg::Drawable* drawable)
    {
        return drawable && drawable->getName() == kInlineMeshDrawableName;
    }

    void disableVirtualProgramInheritance(osg::StateSet* stateSet)
    {
        if (!stateSet)
        {
            return;
        }
        osgEarth::VirtualProgram::getOrCreate(stateSet)->setInheritShaders(false);
    }

    class VirtualProgramIsolationVisitor : public osg::NodeVisitor
    {
    public:
        VirtualProgramIsolationVisitor() :
            osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
        {
        }

        void apply(osg::Node& node) override
        {
            if (dynamic_cast<osgEarth::LineDrawable*>(&node) != nullptr)
            {
                return;
            }
            disableVirtualProgramInheritance(node.getOrCreateStateSet());
            traverse(node);
        }

        void apply(osg::Geode& geode) override
        {
            disableVirtualProgramInheritance(geode.getOrCreateStateSet());
            for (unsigned int i = 0u; i < geode.getNumDrawables(); ++i)
            {
                osg::Drawable* drawable = geode.getDrawable(i);
                if (drawable)
                {
                    if (dynamic_cast<osgEarth::LineDrawable*>(drawable) == nullptr &&
                        !isInlineEdgeDrawable(drawable) &&
                        !isInlineMeshDrawable(drawable))
                    {
                        disableVirtualProgramInheritance(drawable->getOrCreateStateSet());
                    }
                }
            }
            traverse(geode);
        }
    };

    std::uint64_t edgeKey(unsigned int a, unsigned int b)
    {
        const unsigned int lo = std::min(a, b);
        const unsigned int hi = std::max(a, b);
        return (static_cast<std::uint64_t>(lo) << 32u) | static_cast<std::uint64_t>(hi);
    }

    void addUndirectedEdge(
        std::unordered_map<std::uint64_t, EdgeUse>& edges,
        unsigned int a,
        unsigned int b)
    {
        if (a == b)
        {
            return;
        }
        EdgeUse& use = edges[edgeKey(a, b)];
        if (use.count == 0u)
        {
            use.a = std::min(a, b);
            use.b = std::max(a, b);
        }
        ++use.count;
    }

    osg::Program* createInlineEdgeProgram()
    {
        const char* vertexShader = R"(
            #version 330

            uniform mat4 osg_ModelViewProjectionMatrix;
            in vec4 osg_Vertex;

            void main()
            {
                gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;
            }
        )";

        const char* geometryShader = R"(
            #version 330

            layout(lines) in;
            layout(triangle_strip, max_vertices = 4) out;

            uniform float oe_h3dt_edge_width;
            uniform vec2 oe_h3dt_viewport_size;

            void emitOffsetVertex(vec4 clipPosition, vec2 offset)
            {
                gl_Position = clipPosition;
                gl_Position.xy += offset * clipPosition.w;
                EmitVertex();
            }

            void main()
            {
                vec4 p0 = gl_in[0].gl_Position;
                vec4 p1 = gl_in[1].gl_Position;
                vec2 ndc0 = p0.xy / p0.w;
                vec2 ndc1 = p1.xy / p1.w;
                vec2 dir = ndc1 - ndc0;
                if (dot(dir, dir) < 1e-12)
                {
                    return;
                }

                vec2 normal = normalize(vec2(-dir.y, dir.x));
                vec2 viewportSize = max(oe_h3dt_viewport_size, vec2(1.0));
                vec2 offset = normal * max(oe_h3dt_edge_width, 1.0) / viewportSize;

                emitOffsetVertex(p0, offset);
                emitOffsetVertex(p0, -offset);
                emitOffsetVertex(p1, offset);
                emitOffsetVertex(p1, -offset);
                EndPrimitive();
            }
        )";

        const char* fragmentShader = R"(
            #version 330

            uniform vec4 oe_h3dt_edge_color;
            layout(location = 0) out vec4 fragColor;

            void main()
            {
                fragColor = oe_h3dt_edge_color;
            }
        )";

        osg::Program* program = new osg::Program();
        program->setName("h3dt_inline_edge_program");
        program->addShader(new osg::Shader(osg::Shader::VERTEX, vertexShader));
        program->addShader(new osg::Shader(osg::Shader::GEOMETRY, geometryShader));
        program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentShader));
        program->addBindFragDataLocation("fragColor", 0);
        return program;
    }

    osg::Program* createInlineMeshProgram()
    {
        const char* vertexShader = R"(
            #version 330

            uniform mat4 osg_ModelViewProjectionMatrix;
            in vec4 osg_Vertex;

            void main()
            {
                gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;
            }
        )";

        const char* fragmentShader = R"(
            #version 330

            uniform vec4 oe_h3dt_mesh_color;
            layout(location = 0) out vec4 fragColor;

            void main()
            {
                fragColor = oe_h3dt_mesh_color;
            }
        )";

        osg::Program* program = new osg::Program();
        program->setName("h3dt_inline_mesh_program");
        program->addShader(new osg::Shader(osg::Shader::VERTEX, vertexShader));
        program->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragmentShader));
        program->addBindFragDataLocation("fragColor", 0);
        return program;
    }

    template<typename IndexArrayT>
    void collectTriangleBoundaryEdges(
        const IndexArrayT* indices,
        std::unordered_map<std::uint64_t, EdgeUse>& edges)
    {
        if (!indices)
        {
            return;
        }
        for (std::size_t i = 0; i + 2 < indices->size(); i += 3)
        {
            const unsigned int a = static_cast<unsigned int>((*indices)[i + 0]);
            const unsigned int b = static_cast<unsigned int>((*indices)[i + 1]);
            const unsigned int c = static_cast<unsigned int>((*indices)[i + 2]);
            addUndirectedEdge(edges, a, b);
            addUndirectedEdge(edges, b, c);
            addUndirectedEdge(edges, c, a);
        }
    }

    bool parsePositiveInt(const std::string& s, int* out)
    {
        if (s.empty() || !out)
        {
            return false;
        }
        char* end = nullptr;
        const long v = std::strtol(s.c_str(), &end, 10);
        if (end == s.c_str() || *end != '\0' || v < 0 || v > std::numeric_limits<int>::max())
        {
            return false;
        }
        *out = static_cast<int>(v);
        return true;
    }

    bool parseTileIdFromUrl(const std::string& url, TileId* out)
    {
        if (!out)
        {
            return false;
        }
        std::string path = url;
        const std::size_t q = path.find_first_of("?#");
        if (q != std::string::npos)
        {
            path.resize(q);
        }
        for (char& c : path)
        {
            if (c == '\\')
            {
                c = '/';
            }
        }
        const std::size_t dot = path.rfind(".glb");
        if (dot == std::string::npos)
        {
            return false;
        }
        const std::size_t yStart = path.rfind('/', dot);
        if (yStart == std::string::npos)
        {
            return false;
        }
        const std::size_t xEnd = yStart;
        const std::size_t xStart = path.rfind('/', xEnd - 1);
        if (xStart == std::string::npos)
        {
            return false;
        }
        const std::size_t zEnd = xStart;
        const std::size_t zStart = path.rfind('/', zEnd - 1);
        if (zStart == std::string::npos)
        {
            return false;
        }

        TileId id;
        if (!parsePositiveInt(path.substr(zStart + 1, zEnd - zStart - 1), &id.z) ||
            !parsePositiveInt(path.substr(xStart + 1, xEnd - xStart - 1), &id.x) ||
            !parsePositiveInt(path.substr(yStart + 1, dot - yStart - 1), &id.y))
        {
            return false;
        }
        *out = id;
        return true;
    }

    TileBoundsRad geodeticTileBounds(const TileId& id)
    {
        constexpr double kPi = 3.14159265358979323846;
        const unsigned tiles = 1u << static_cast<unsigned>(id.z);
        const double lonExtent = 360.0 / static_cast<double>(tiles);
        const double latExtent = 180.0 / static_cast<double>(tiles);
        const double westDeg = -180.0 + static_cast<double>(id.x) * lonExtent;
        const double eastDeg = -180.0 + static_cast<double>(id.x + 1) * lonExtent;
        const double northDeg = 90.0 - static_cast<double>(id.y) * latExtent;
        const double southDeg = 90.0 - static_cast<double>(id.y + 1) * latExtent;
        return TileBoundsRad{
            westDeg * (kPi / 180.0),
            southDeg * (kPi / 180.0),
            eastDeg * (kPi / 180.0),
            northDeg * (kPi / 180.0)};
    }

    void geodeticRadHeightToEcef(
        double lonRad,
        double latRad,
        double heightM,
        double* ox,
        double* oy,
        double* oz)
    {
        constexpr double kA = 6378137.0;
        constexpr double kInvF = 298.257223563;
        constexpr double kF = 1.0 / kInvF;
        constexpr double kE2 = kF * (2.0 - kF);
        const double sinLat = std::sin(latRad);
        const double cosLat = std::cos(latRad);
        const double sinLon = std::sin(lonRad);
        const double cosLon = std::cos(lonRad);
        const double n = kA / std::sqrt(1.0 - kE2 * sinLat * sinLat);
        *ox = (n + heightM) * cosLat * cosLon;
        *oy = (n + heightM) * cosLat * sinLon;
        *oz = (n * (1.0 - kE2) + heightM) * sinLat;
    }

    void ecefToGeodeticRadHeight(
        double x,
        double y,
        double z,
        double* lonRad,
        double* latRad,
        double* heightM)
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kA = 6378137.0;
        constexpr double kInvF = 298.257223563;
        constexpr double kF = 1.0 / kInvF;
        constexpr double kB = kA * (1.0 - kF);
        constexpr double kE2 = kF * (2.0 - kF);
        constexpr double kEp2 = (kA * kA - kB * kB) / (kB * kB);

        const double p = std::sqrt(x * x + y * y);
        *lonRad = std::atan2(y, x);
        if (p <= std::numeric_limits<double>::epsilon())
        {
            *latRad = (z >= 0.0) ? (kPi * 0.5) : (-kPi * 0.5);
            *heightM = std::fabs(z) - kB;
            return;
        }

        const double theta = std::atan2(z * kA, p * kB);
        const double sinTheta = std::sin(theta);
        const double cosTheta = std::cos(theta);
        *latRad = std::atan2(
            z + kEp2 * kB * sinTheta * sinTheta * sinTheta,
            p - kE2 * kA * cosTheta * cosTheta * cosTheta);
        const double sinLat = std::sin(*latRad);
        const double n = kA / std::sqrt(1.0 - kE2 * sinLat * sinLat);
        *heightM = p / std::cos(*latRad) - n;
    }

    double unwrapLonNear(double lonRad, double referenceRad)
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kTwoPi = 6.28318530717958647692;
        while (lonRad - referenceRad > kPi)
        {
            lonRad -= kTwoPi;
        }
        while (lonRad - referenceRad < -kPi)
        {
            lonRad += kTwoPi;
        }
        return lonRad;
    }

    TileLocalFrame makeTileLocalFrame(const TileId& id)
    {
        TileLocalFrame out;
        out.valid = id.z >= 0;
        if (!out.valid)
        {
            return out;
        }
        out.bounds = geodeticTileBounds(id);
        const double lonC = (out.bounds.west + out.bounds.east) * 0.5;
        const double latC = (out.bounds.south + out.bounds.north) * 0.5;
        const double sinLon = std::sin(lonC);
        const double cosLon = std::cos(lonC);
        const double sinLat = std::sin(latC);
        const double cosLat = std::cos(latC);

        geodeticRadHeightToEcef(
            out.bounds.west, out.bounds.south, 0.0,
            &out.origin[0], &out.origin[1], &out.origin[2]);

        out.east[0] = -sinLon;
        out.east[1] = cosLon;
        out.east[2] = 0.0;
        out.north[0] = -sinLat * cosLon;
        out.north[1] = -sinLat * sinLon;
        out.north[2] = cosLat;
        out.up[0] = cosLat * cosLon;
        out.up[1] = cosLat * sinLon;
        out.up[2] = sinLat;
        return out;
    }

    bool gltfPointToLonLat(
        const osg::Vec3& p,
        const TileLocalFrame& frame,
        double* lonRad,
        double* latRad)
    {
        if (!frame.valid || !lonRad || !latRad)
        {
            return false;
        }
        const double eastM = static_cast<double>(p.x());
        const double upM = static_cast<double>(p.y());
        const double northM = -static_cast<double>(p.z());
        const double ecefX = frame.origin[0] + eastM * frame.east[0] +
            northM * frame.north[0] + upM * frame.up[0];
        const double ecefY = frame.origin[1] + eastM * frame.east[1] +
            northM * frame.north[1] + upM * frame.up[1];
        const double ecefZ = frame.origin[2] + eastM * frame.east[2] +
            northM * frame.north[2] + upM * frame.up[2];
        double heightM = 0.0;
        ecefToGeodeticRadHeight(ecefX, ecefY, ecefZ, lonRad, latRad, &heightM);
        *lonRad = unwrapLonNear(*lonRad, (frame.bounds.west + frame.bounds.east) * 0.5);
        return std::isfinite(*lonRad) && std::isfinite(*latRad);
    }

    bool gltfPointToLonLatHeight(
        const osg::Vec3& p,
        const TileLocalFrame& frame,
        double* lonRad,
        double* latRad,
        double* heightM)
    {
        if (!frame.valid || !lonRad || !latRad || !heightM)
        {
            return false;
        }
        const double eastM = static_cast<double>(p.x());
        const double upM = static_cast<double>(p.y());
        const double northM = -static_cast<double>(p.z());
        const double ecefX = frame.origin[0] + eastM * frame.east[0] +
            northM * frame.north[0] + upM * frame.up[0];
        const double ecefY = frame.origin[1] + eastM * frame.east[1] +
            northM * frame.north[1] + upM * frame.up[1];
        const double ecefZ = frame.origin[2] + eastM * frame.east[2] +
            northM * frame.north[2] + upM * frame.up[2];
        ecefToGeodeticRadHeight(ecefX, ecefY, ecefZ, lonRad, latRad, heightM);
        *lonRad = unwrapLonNear(*lonRad, (frame.bounds.west + frame.bounds.east) * 0.5);
        return std::isfinite(*lonRad) && std::isfinite(*latRad) && std::isfinite(*heightM);
    }

    bool lonLatHeightToGltfPoint(
        double lonRad,
        double latRad,
        double heightM,
        const TileLocalFrame& frame,
        osg::Vec3* out)
    {
        if (!frame.valid || !out ||
            !std::isfinite(lonRad) || !std::isfinite(latRad) || !std::isfinite(heightM))
        {
            return false;
        }

        double ecef[3]{};
        geodeticRadHeightToEcef(lonRad, latRad, heightM, &ecef[0], &ecef[1], &ecef[2]);
        const double dx = ecef[0] - frame.origin[0];
        const double dy = ecef[1] - frame.origin[1];
        const double dz = ecef[2] - frame.origin[2];
        const double eastM = dx * frame.east[0] + dy * frame.east[1] + dz * frame.east[2];
        const double northM = dx * frame.north[0] + dy * frame.north[1] + dz * frame.north[2];
        const double upM = dx * frame.up[0] + dy * frame.up[1] + dz * frame.up[2];
        if (!std::isfinite(eastM) || !std::isfinite(northM) || !std::isfinite(upM))
        {
            return false;
        }
        out->set(
            static_cast<float>(eastM),
            static_cast<float>(upM),
            static_cast<float>(-northM));
        return true;
    }

    bool terrainHeightSampleValid(double z)
    {
        return std::isfinite(z) && z > -100000.0;
    }

    bool applyTerrainClampToGeometry(
        osg::Geometry* geom,
        const TilesetRenderStyleOptions* renderStyle,
        const osg::Matrixd& localToEcef)
    {
        if (!geom || !renderStyle || !renderStyle->clampToGround ||
            !renderStyle->clampMap)
        {
            return false;
        }

        const osg::Vec3Array* sourcePositions = dynamic_cast<const osg::Vec3Array*>(geom->getVertexArray());
        if (!sourcePositions || sourcePositions->empty())
        {
            return false;
        }

        osg::Matrixd ecefToLocal;
        if (!ecefToLocal.invert(localToEcef))
        {
            return false;
        }

        osg::ref_ptr<const osgEarth::SpatialReference> wgs84 = osgEarth::SpatialReference::get("wgs84");
        if (!wgs84.valid())
        {
            return false;
        }

        constexpr double kQuantScale = 1.0e6;
        const unsigned int maxSamples = std::max(renderStyle->clampMaxTerrainSamples, 1u);

        osg::ref_ptr<osg::Vec3Array> clampedPositions = new osg::Vec3Array(*sourcePositions);
        std::vector<std::size_t> vertexToSample(clampedPositions->size(), std::numeric_limits<std::size_t>::max());
        std::vector<TerrainSample> samples;
        samples.reserve(std::min<std::size_t>(clampedPositions->size(), maxSamples));
        std::unordered_map<TerrainSampleKey, std::size_t, TerrainSampleKeyHash> sampleIndexByKey;
        sampleIndexByKey.reserve(samples.capacity());

        for (std::size_t i = 0; i < clampedPositions->size(); ++i)
        {
            const osg::Vec3d ecef =
                osg::Vec3d((*clampedPositions)[i].x(), (*clampedPositions)[i].y(), (*clampedPositions)[i].z()) *
                localToEcef;
            osg::Vec3d llaDeg;
            if (!wgs84->transformFromWorld(ecef, llaDeg) ||
                !std::isfinite(llaDeg.x()) ||
                !std::isfinite(llaDeg.y()) ||
                !std::isfinite(llaDeg.z()))
            {
                continue;
            }

            const TerrainSampleKey key{
                static_cast<std::int64_t>(std::llround(llaDeg.x() * kQuantScale)),
                static_cast<std::int64_t>(std::llround(llaDeg.y() * kQuantScale))};
            const auto found = sampleIndexByKey.find(key);
            if (found != sampleIndexByKey.end())
            {
                vertexToSample[i] = found->second;
                continue;
            }

            if (samples.size() >= maxSamples)
            {
                OE_WARN << "[PrepareRenderResources] Skipping 3D Tiles terrain clamp: sample budget exceeded ("
                    << maxSamples << ")\n";
                return false;
            }

            const std::size_t sampleIndex = samples.size();
            TerrainSample sample;
            sample.queryPoint.set(llaDeg.x(), llaDeg.y(), 0.0);
            samples.push_back(sample);
            sampleIndexByKey.emplace(key, sampleIndex);
            vertexToSample[i] = sampleIndex;
        }

        if (samples.empty())
        {
            return false;
        }

        osgEarth::Util::ElevationQuery query(renderStyle->clampMap);
        std::vector<osg::Vec3d> queryPoints;
        queryPoints.reserve(samples.size());
        for (const TerrainSample& sample : samples)
        {
            queryPoints.push_back(sample.queryPoint);
        }

        const double desiredResolution =
            renderStyle->clampSampleResolutionM > 0.0 ? renderStyle->clampSampleResolutionM : 0.0;
        constexpr std::size_t kChunkSize = 2048u;
        for (std::size_t offset = 0; offset < queryPoints.size(); offset += kChunkSize)
        {
            const std::size_t count = std::min(kChunkSize, queryPoints.size() - offset);
            std::vector<osg::Vec3d> chunk(
                queryPoints.begin() + static_cast<std::ptrdiff_t>(offset),
                queryPoints.begin() + static_cast<std::ptrdiff_t>(offset + count));
            if (!query.getElevations(chunk, wgs84.get(), true, desiredResolution))
            {
                return false;
            }
            for (std::size_t i = 0; i < chunk.size(); ++i)
            {
                samples[offset + i].queryPoint.z() = chunk[i].z();
            }
        }

        bool wroteAny = false;
        for (std::size_t i = 0; i < clampedPositions->size(); ++i)
        {
            const std::size_t sampleIndex = vertexToSample[i];
            if (sampleIndex >= samples.size())
            {
                continue;
            }
            const TerrainSample& sample = samples[sampleIndex];
            const double terrainHeightM = sample.queryPoint.z();
            if (!terrainHeightSampleValid(terrainHeightM))
            {
                continue;
            }

            osgEarth::GeoPoint clampedPoint(
                wgs84.get(),
                sample.queryPoint.x(),
                sample.queryPoint.y(),
                terrainHeightM,
                osgEarth::ALTMODE_ABSOLUTE);
            osg::Vec3d clampedEcef;
            if (!clampedPoint.toWorld(clampedEcef))
                continue;

            const osg::Vec3d local = clampedEcef * ecefToLocal;
            if (!std::isfinite(local.x()) || !std::isfinite(local.y()) || !std::isfinite(local.z()))
                continue;

            (*clampedPositions)[i].set(
                static_cast<float>(local.x()),
                static_cast<float>(local.y()),
                static_cast<float>(local.z()));
            wroteAny = true;
        }

        if (wroteAny)
        {
            geom->setVertexArray(clampedPositions.get());
            geom->dirtyBound();
        }
        return wroteAny;
    }

    class TerrainClampVisitor : public osg::NodeVisitor
    {
    public:
        TerrainClampVisitor(const TilesetRenderStyleOptions* renderStyle) :
            osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
            _renderStyle(renderStyle)
        {
            _matrixStack.push_back(osg::Matrixd::identity());
        }

        void apply(osg::MatrixTransform& transform) override
        {
            osg::Matrixd localToEcef = currentMatrix();
            transform.computeLocalToWorldMatrix(localToEcef, this);
            _matrixStack.push_back(localToEcef);
            traverse(transform);
            _matrixStack.pop_back();
        }

        void apply(osg::Geometry& geometry) override
        {
            if (applyTerrainClampToGeometry(&geometry, _renderStyle, currentMatrix()))
            {
                ++_clampedGeometryCount;
            }
            traverse(geometry);
        }

        unsigned int clampedGeometryCount() const
        {
            return _clampedGeometryCount;
        }

    private:
        const osg::Matrixd& currentMatrix() const
        {
            return _matrixStack.back();
        }

        const TilesetRenderStyleOptions* _renderStyle = nullptr;
        std::vector<osg::Matrixd> _matrixStack;
        unsigned int _clampedGeometryCount = 0u;
    };

    unsigned int applyTerrainClampToSubgraph(
        osg::Node* node,
        const TilesetRenderStyleOptions* renderStyle)
    {
        if (!node || !renderStyle || !renderStyle->clampToGround)
        {
            return 0u;
        }

        TerrainClampVisitor visitor(renderStyle);
        node->accept(visitor);
        return visitor.clampedGeometryCount();
    }

    bool snapshotTerrainClampGeometry(
        osg::Geometry* geom,
        const TilesetRenderStyleOptions& renderStyle,
        const osg::Matrixd& localToEcef,
        TilesetTerrainClampGeometry& out)
    {
        if (!geom || !renderStyle.clampToGround || !renderStyle.clampMap)
            return false;

        osg::Matrixd ecefToLocal;
        if (!ecefToLocal.invert(localToEcef))
            return false;

        osg::ref_ptr<const osgEarth::SpatialReference> wgs84 = osgEarth::SpatialReference::get("wgs84");
        if (!wgs84.valid())
            return false;

        const osg::Vec3Array* sourcePositions = dynamic_cast<const osg::Vec3Array*>(geom->getVertexArray());
        if (!sourcePositions || sourcePositions->empty())
            return false;

        constexpr double kQuantScale = 1.0e6;
        const unsigned int maxSamples = std::max(renderStyle.clampMaxTerrainSamples, 1u);
        osg::ref_ptr<osg::Vec3Array> positionsCopy = new osg::Vec3Array(*sourcePositions);

        std::vector<std::size_t> vertexToSample(positionsCopy->size(), std::numeric_limits<std::size_t>::max());
        std::vector<osg::Vec3d> samples;
        samples.reserve(std::min<std::size_t>(positionsCopy->size(), maxSamples));
        std::unordered_map<TerrainSampleKey, std::size_t, TerrainSampleKeyHash> sampleIndexByKey;
        sampleIndexByKey.reserve(samples.capacity());

        for (std::size_t i = 0; i < positionsCopy->size(); ++i)
        {
            const osg::Vec3d ecef =
                osg::Vec3d((*positionsCopy)[i].x(), (*positionsCopy)[i].y(), (*positionsCopy)[i].z()) *
                localToEcef;
            osg::Vec3d llaDeg;
            if (!wgs84->transformFromWorld(ecef, llaDeg) ||
                !std::isfinite(llaDeg.x()) ||
                !std::isfinite(llaDeg.y()) ||
                !std::isfinite(llaDeg.z()))
            {
                continue;
            }

            const TerrainSampleKey key{
                static_cast<std::int64_t>(std::llround(llaDeg.x() * kQuantScale)),
                static_cast<std::int64_t>(std::llround(llaDeg.y() * kQuantScale))};
            const auto found = sampleIndexByKey.find(key);
            if (found != sampleIndexByKey.end())
            {
                vertexToSample[i] = found->second;
                continue;
            }

            if (samples.size() >= maxSamples)
                return false;

            const std::size_t sampleIndex = samples.size();
            samples.emplace_back(llaDeg.x(), llaDeg.y(), 0.0);
            sampleIndexByKey.emplace(key, sampleIndex);
            vertexToSample[i] = sampleIndex;
        }

        if (samples.empty())
            return false;

        out.geometry = geom;
        out.sourcePositions = positionsCopy.get();
        out.ecefToLocal = ecefToLocal;
        out.vertexToSample = std::move(vertexToSample);
        out.samplesLonLatHeight = std::move(samples);
        out.resolved = false;
        return true;
    }

    class TerrainClampSnapshotVisitor : public osg::NodeVisitor
    {
    public:
        TerrainClampSnapshotVisitor(const TilesetRenderStyleOptions& renderStyle, TilesetTerrainClampTask& task) :
            osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
            _renderStyle(renderStyle),
            _task(task)
        {
            _matrixStack.push_back(osg::Matrixd::identity());
        }

        void apply(osg::MatrixTransform& transform) override
        {
            osg::Matrixd localToEcef = _matrixStack.back();
            transform.computeLocalToWorldMatrix(localToEcef, this);
            _matrixStack.push_back(localToEcef);
            traverse(transform);
            _matrixStack.pop_back();
        }

        void apply(osg::Geometry& geometry) override
        {
            TilesetTerrainClampGeometry snapshot;
            if (snapshotTerrainClampGeometry(&geometry, _renderStyle, _matrixStack.back(), snapshot))
                _task.geometries.push_back(std::move(snapshot));
            traverse(geometry);
        }

    private:
        const TilesetRenderStyleOptions& _renderStyle;
        TilesetTerrainClampTask& _task;
        std::vector<osg::Matrixd> _matrixStack;
    };

    bool isTileGridEdge(
        const osg::Vec3& a,
        const osg::Vec3& b,
        const TileLocalFrame& frame)
    {
        double lonA = 0.0, latA = 0.0, lonB = 0.0, latB = 0.0;
        if (!gltfPointToLonLat(a, frame, &lonA, &latA) ||
            !gltfPointToLonLat(b, frame, &lonB, &latB))
        {
            return false;
        }

        const double lonSpan = std::max(std::fabs(frame.bounds.east - frame.bounds.west), 1e-15);
        const double latSpan = std::max(std::fabs(frame.bounds.north - frame.bounds.south), 1e-15);
        const double lonEps = std::max(lonSpan * 1e-5, 1e-9);
        const double latEps = std::max(latSpan * 1e-5, 1e-9);
        const bool onWest = std::fabs(lonA - frame.bounds.west) <= lonEps &&
            std::fabs(lonB - frame.bounds.west) <= lonEps;
        const bool onEast = std::fabs(lonA - frame.bounds.east) <= lonEps &&
            std::fabs(lonB - frame.bounds.east) <= lonEps;
        const bool onSouth = std::fabs(latA - frame.bounds.south) <= latEps &&
            std::fabs(latB - frame.bounds.south) <= latEps;
        const bool onNorth = std::fabs(latA - frame.bounds.north) <= latEps &&
            std::fabs(latB - frame.bounds.north) <= latEps;
        return onWest || onEast || onSouth || onNorth;
    }

    #ifdef GL_R
    const GLenum redFormat = GL_R;
    #else
    const GLenum redFormat = GL_RED;
    #endif

    osg::Image* getOsgImage(CesiumGltf::ImageAsset& image)
    {
        GLenum format = GL_RGB;
        GLenum texFormat = GL_RGB8;
        GLenum type = GL_UNSIGNED_BYTE;

        switch (image.compressedPixelFormat)
        {
        case CesiumGltf::GpuCompressedPixelFormat::NONE:
            switch (image.channels)
            {
            case 1:
                format = redFormat;
                texFormat = GL_R8;
                break;
            case 2:
                format = GL_RG;
                texFormat = GL_RG8;
                break;
            case 3:
                format = GL_RGB;
                texFormat = GL_RGB8;
                break;
            case 4:
                format = GL_RGBA;
                texFormat = GL_RGBA8;
                break;
            }
            break;
        case CesiumGltf::GpuCompressedPixelFormat::ETC1_RGB:
            format = GL_RGB;
            texFormat = GL_ETC1_RGB8_OES;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::ETC2_RGBA:
            format = GL_RGBA;
            texFormat = GL_COMPRESSED_RGBA8_ETC2_EAC;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::BC1_RGB:
            format = GL_RGB;
            texFormat = GL_COMPRESSED_RGB_S3TC_DXT1_EXT;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::BC3_RGBA:
            format = GL_RGB;
            texFormat = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::BC4_R:
            format = redFormat;
            texFormat = GL_COMPRESSED_RED_RGTC1_EXT;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::BC5_RG:
            format = GL_RG;
            texFormat = GL_COMPRESSED_RED_GREEN_RGTC2_EXT;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::BC7_RGBA:
            OE_NOTICE << "Unsuported compressed format BC7_RGBA" << std::endl;
            return nullptr;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::PVRTC1_4_RGB:
            format = GL_RGB;
            texFormat = GL_COMPRESSED_RGB_PVRTC_4BPPV1_IMG;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::PVRTC1_4_RGBA:
            format = GL_RGBA;
            texFormat = GL_COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::ASTC_4x4_RGBA:
            format = GL_RGBA;
            texFormat = GL_KHR_texture_compression_astc_hdr;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::PVRTC2_4_RGB:
            OE_NOTICE << "Unsuported compressed format PVRTC2_4_RGB" << std::endl;
            return nullptr;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::PVRTC2_4_RGBA:
            OE_NOTICE << "Unsuported compressed format PVRTC2_4_RGBA" << std::endl;
            return nullptr;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::ETC2_EAC_R11:
            format = redFormat;
            texFormat = GL_COMPRESSED_R11_EAC;
            break;
        case CesiumGltf::GpuCompressedPixelFormat::ETC2_EAC_RG11:
            format = GL_RG;
            texFormat = GL_COMPRESSED_RG11_EAC;
            break;
        default:
            break;
        }

        osg::Image* osgImage = new osg::Image;
        unsigned char* imgData = new unsigned char[image.pixelData.size()];
        memcpy(imgData, &image.pixelData[0], image.pixelData.size());
        osgImage->setImage(image.width, image.height, 1, texFormat, format, type, imgData, osg::Image::AllocationMode::USE_NEW_DELETE);
        return osgImage;
    }
}


LoadThreadResult::LoadThreadResult()
{
}

LoadThreadResult::~LoadThreadResult()
{    
}

MainThreadResult::MainThreadResult()
{
}

MainThreadResult::~MainThreadResult()
{
}


LoadRasterThreadResult::LoadRasterThreadResult()
{
}

LoadRasterThreadResult::~LoadRasterThreadResult()
{
}

LoadRasterMainThreadResult::LoadRasterMainThreadResult()
{
}

LoadRasterMainThreadResult::~LoadRasterMainThreadResult()
{
}


/********/
namespace {

    template<typename T>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<T>& accessorView)
    {
        return nullptr;
    }

    // TODO:  The rest of the accessor types.

    template<>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<CesiumGltf::AccessorTypes::SCALAR<uint16_t>>& accessorView)
    {
        osg::UShortArray* result = new osg::UShortArray(accessorView.size());
        for (unsigned int i = 0; i < accessorView.size(); ++i)
        {
            auto& data = accessorView[i];
            (*result)[i] = data.value[0];
        }
        return result;
    }

    template<>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<CesiumGltf::AccessorTypes::SCALAR<uint32_t>>& accessorView)
    {
        osg::UIntArray* result = new osg::UIntArray(accessorView.size());
        for (unsigned int i = 0; i < accessorView.size(); ++i)
        {
            auto& data = accessorView[i];
            (*result)[i] = data.value[0];
        }
        return result;
    }

    template<>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<CesiumGltf::AccessorTypes::SCALAR<uint8_t>>& accessorView)
    {
        osg::UByteArray* result = new osg::UByteArray(accessorView.size());
        for (unsigned int i = 0; i < accessorView.size(); ++i)
        {
            auto& data = accessorView[i];
            (*result)[i] = data.value[0];
        }
        return result;
    }

    template<>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<CesiumGltf::AccessorTypes::VEC3<float>>& accessorView)
    {
        osg::Vec3Array* result = new osg::Vec3Array(accessorView.size());
        for (unsigned int i = 0; i < accessorView.size(); ++i)
        {
            auto& data = accessorView[i];
            (*result)[i].set(data.value[0], data.value[1], data.value[2]);
        }
        return result;
    }

    template<>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<CesiumGltf::AccessorTypes::VEC2<float>>& accessorView)
    {
        osg::Vec2Array* result = new osg::Vec2Array(accessorView.size());
        for (unsigned int i = 0; i < accessorView.size(); ++i)
        {
            auto& data = accessorView[i];
            (*result)[i].set(data.value[0], data.value[1]);
        }
        return result;
    }

    template<>
    osg::Array* accessorViewToArray(const CesiumGltf::AccessorView<CesiumGltf::AccessorTypes::SCALAR<float>>& accessorView)
    {
        osg::FloatArray* result = new osg::FloatArray(accessorView.size());
        for (unsigned int i = 0; i < accessorView.size(); ++i)
        {
            auto& data = accessorView[i];
            (*result)[i] = data.value[0];
        }
        return result;
    }

    class NodeBuilder
    {
    public:
        NodeBuilder(
            CesiumGltf::Model* model,
            const glm::dmat4& transform,
            const TilesetRenderStyleOptions* renderStyle,
            const TileLocalFrame& tileFrame) :
            _model(model),
            _transform(transform),
            _renderStyle(renderStyle),
            _tileFrame(tileFrame)
        {
            loadArrays();
            loadTextures();        
        }

        osg::Node* build()
        {
            osg::MatrixTransform* root = new osg::MatrixTransform;
            osg::Matrixd matrix;

            glm::dmat4x4 rootTransform = _transform;
            rootTransform = CesiumGltfContent::GltfUtilities::applyRtcCenter(*_model, rootTransform);
            rootTransform = CesiumGltfContent::GltfUtilities::applyGltfUpAxisTransform(*_model, rootTransform);
            matrix.set(glm::value_ptr(rootTransform));

            //matrix.set(glm::value_ptr(_transform));
            root->setMatrix(matrix);
        
            if (!_model->scenes.empty())
            {
                for (auto& scene : _model->scenes)
                {
                    for (int node : scene.nodes)
                    {
                        root->addChild(createNode(_model->nodes[node]));
                    }
                }
            }
            else
            {
                for (auto itr = _model->nodes.begin(); itr != _model->nodes.end(); ++itr)
                {
                    root->addChild(createNode(*itr));
                }
            }

            osgEarth::Registry::shaderGenerator().run(root);
            VirtualProgramIsolationVisitor isolateVirtualPrograms;
            root->accept(isolateVirtualPrograms);
            osg::Group* container = new osg::Group;
            disableVirtualProgramInheritance(container->getOrCreateStateSet());
            container->addChild(root);
            return container;
        }

        osg::Node* createNode(const CesiumGltf::Node& node)
        {
            osg::MatrixTransform* root = new osg::MatrixTransform;
            if (node.matrix.size() == 16)
            {
                osg::Matrixd matrix;
                matrix.set(node.matrix.data());
                root->setMatrix(matrix);
            }


            if (root->getMatrix().isIdentity())
            {
                osg::Matrixd scale, translation, rotation;
                if (node.scale.size() == 3)
                {
                    scale = osg::Matrixd::scale(node.scale[0], node.scale[1], node.scale[2]);
                }

                if (node.rotation.size() == 4) {
                    osg::Quat quat(node.rotation[0], node.rotation[1], node.rotation[2], node.rotation[3]);
                    rotation.makeRotate(quat);
                }

                if (node.translation.size() == 3) {
                    translation = osg::Matrixd::translate(node.translation[0], node.translation[1], node.translation[2]);
                }

                root->setMatrix(scale * rotation * translation);
            }

            if (node.mesh >= 0)
            {
                // Build the mesh and add it.
                // TODO:  This mesh needs cached since it can be reused and referenced elsewhere.
                root->addChild(createMesh(_model->meshes[node.mesh]));
            }

            for (int child : node.children)
            {
                root->addChild(createNode(_model->nodes[child]));
            }

            return root;
        }    

        void loadArrays()
        {
            for (unsigned int i = 0; i < _model->accessors.size(); ++i)
            {
                osg::ref_ptr< osg::Array > osgArray = nullptr;
                CesiumGltf::createAccessorView(*this->_model, i, [&](const auto& accessorView) {
                    osgArray = accessorViewToArray(accessorView);
                    });

                if (osgArray)
                {
                    osgArray->setBinding(osg::Array::BIND_PER_VERTEX);
                    osgArray->setNormalize(_model->accessors[i].normalized);
                    _arrays.push_back(osgArray);
                }
                else
                {
                    _arrays.push_back(nullptr);
                }
            }
        }

        void loadTextures()
        {
            for (unsigned int i = 0; i < _model->textures.size(); ++i)
            {
                auto& texture = _model->textures[i];

                osg::Texture2D* osgTexture = new osg::Texture2D;
                osgTexture->setResizeNonPowerOfTwoHint(false);
                osgTexture->setDataVariance(osg::Object::STATIC);
                // Sampler settings
                if (texture.sampler >= 0 && texture.sampler < _model->samplers.size())
                {
                    auto& sampler = _model->samplers[texture.sampler];

                    // TODO?  Actually set filter?
                    osgTexture->setFilter(osg::Texture::MIN_FILTER, (osg::Texture::FilterMode)osg::Texture::LINEAR_MIPMAP_LINEAR); //sampler.minFilter);
                    osgTexture->setFilter(osg::Texture::MAG_FILTER, (osg::Texture::FilterMode)osg::Texture::LINEAR); //sampler.magFilter);
                    osgTexture->setWrap(osg::Texture::WRAP_S, (osg::Texture::WrapMode)sampler.wrapS);
                    osgTexture->setWrap(osg::Texture::WRAP_T, (osg::Texture::WrapMode)sampler.wrapT);
                    //osgTexture->setWrap(osg::Texture::WRAP_R, (osg::Texture::WrapMode)sampler.wrapR);
                }
                else
                {
                    osgTexture->setFilter(osg::Texture::MIN_FILTER, (osg::Texture::FilterMode)osg::Texture::LINEAR_MIPMAP_LINEAR);
                    osgTexture->setFilter(osg::Texture::MAG_FILTER, (osg::Texture::FilterMode)osg::Texture::LINEAR);
                    osgTexture->setWrap(osg::Texture::WRAP_S, (osg::Texture::WrapMode)osg::Texture::CLAMP_TO_EDGE);
                    osgTexture->setWrap(osg::Texture::WRAP_T, (osg::Texture::WrapMode)osg::Texture::CLAMP_TO_EDGE);
                }

                // Load the image
                auto& image = *_model->images[texture.source].pAsset;

                osg::Image* osgImage = getOsgImage(image);

                if (osgImage)
                {
                    osgTexture->setImage(osgImage);
                }

                _textures.push_back(osgTexture);
            }
        }

        void applyTerrainClamp(osg::Geometry* geom)
        {
        }

        osg::Node* createMesh(const CesiumGltf::Mesh& mesh)
        {
            osg::Group* geode = new osg::Group;
            for (auto primitive : mesh.primitives)
            {
                osg::ref_ptr< osg::Geometry> geom = new osg::Geometry;
                geom->setUseDisplayList(false);
                geom->setUseVertexBufferObjects(true);

                for (auto& attribute : primitive.attributes)
                {
                    const std::string& name = attribute.first;
                    osg::Array* osgArray = _arrays[attribute.second];
                    if (name == "POSITION")
                    {
                        geom->setVertexArray(osgArray);
                    }
                    else if (name == "COLOR_0")
                    {
                        geom->setColorArray(osgArray);
                    }
                    else if (name == "NORMAL")
                    {
                        geom->setNormalArray(osgArray);
                    }
                    else if (name == "TEXCOORD_0")
                    {
                        geom->setTexCoordArray(0, osgArray);
                    }
                    else if (name == "TEXCOORD_1")
                    {
                        geom->setTexCoordArray(1, osgArray);
                    }

                    // Look for Cesium Overlay texture coordinates.
                    const std::string CESIUM_OVERLAY = "_CESIUMOVERLAY_";
                    if (osgEarth::startsWith(name, CESIUM_OVERLAY))
                    {
                        int index = std::stoi(name.substr(CESIUM_OVERLAY.length()));

                        // Not sure how many overlays or tex coords we should support.  For now we'll support 2 texture coords and two overlays so stick the 
                        // overlay texture coordinates at an index of index + 2.  Revisit this later.
                        geom->setTexCoordArray(index + 2, osgArray);
                    }
                }

                // If there is no color array just add one
                if (!geom->getColorArray())
                {
                    osg::Vec4Array* colors = new osg::Vec4Array();
                    osg::Vec3Array* verts = static_cast<osg::Vec3Array*>(geom->getVertexArray());
                    for (unsigned int i = 0; i < verts->size(); i++)
                    {
                        colors->push_back(osg::Vec4(1, 1, 1, 1));
                    }
                    geom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
                }

                GLenum mode = GL_TRIANGLES;
                switch (primitive.mode)
                {
                case CesiumGltf::MeshPrimitive::Mode::TRIANGLES:
                    mode = GL_TRIANGLES;
                    break;
                case CesiumGltf::MeshPrimitive::Mode::TRIANGLE_FAN:
                    mode = GL_TRIANGLE_FAN;
                    break;
                case CesiumGltf::MeshPrimitive::Mode::TRIANGLE_STRIP:
                    mode = GL_TRIANGLE_STRIP;
                    break;
                case CesiumGltf::MeshPrimitive::Mode::LINES:
                    mode = GL_LINES;
                    break;
                case CesiumGltf::MeshPrimitive::Mode::LINE_LOOP:
                    mode = GL_LINES;
                    break;
                case CesiumGltf::MeshPrimitive::Mode::LINE_STRIP:
                    mode = GL_LINE_STRIP;
                    break;
                case CesiumGltf::MeshPrimitive::Mode::POINTS:
                    mode = GL_POINTS;
                    break;
                }

                if (primitive.indices >= 0)
                {
                    osg::Array* primitiveArray = _arrays[primitive.indices];                    

                    switch (primitiveArray->getType())
                    {
                    case osg::Array::UShortArrayType:
                    {
                        osg::UShortArray* indices = static_cast<osg::UShortArray*>(primitiveArray);
                        osg::DrawElementsUShort* drawElements = new osg::DrawElementsUShort(mode, indices->begin(), indices->end());
                        geom->addPrimitiveSet(drawElements);
                        break;
                    }
                    case osg::Array::UIntArrayType:
                    {
                        osg::UIntArray* indices = static_cast<osg::UIntArray*>(primitiveArray);
                        osg::DrawElementsUInt* drawElements
                            = new osg::DrawElementsUInt(mode, indices->begin(), indices->end());
                        geom->addPrimitiveSet(drawElements);
                        break;
                    }
                    case osg::Array::UByteArrayType:
                    {
                        osg::UByteArray* indices = static_cast<osg::UByteArray*>(primitiveArray);
                        // DrawElementsUByte doesn't have the constructor with iterator arguments.
                        osg::DrawElementsUByte* drawElements = new osg::DrawElementsUByte(mode, indices->size());
                        std::copy(indices->begin(), indices->end(), drawElements->begin());
                        geom->addPrimitiveSet(drawElements);
                        break;
                    }
                    }
                }
                else
                {
                    // If there are no primitives, then it is non-indexed geometry                    
                    geom->addPrimitiveSet(new osg::DrawArrays(mode, 0, geom->getVertexArray()->getNumElements()));
                }


                if (primitive.material >= 0 && primitive.material < _model->materials.size())
                {
                    osg::StateSet* stateSet = geom->getOrCreateStateSet();
                    auto& material = _model->materials[primitive.material];
                    auto pbr = material.pbrMetallicRoughness;
                    if (pbr)
                    {
                        if (pbr->baseColorFactor.size() > 0) {
                            osgEarth::MaterialGL3* material = new osgEarth::MaterialGL3();
                            osg::Vec4d color(pbr->baseColorFactor[0], pbr->baseColorFactor[1], pbr->baseColorFactor[2], pbr->baseColorFactor[3]);
                            material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
                            stateSet->setAttributeAndModes(material);
                        }
                        if (pbr->baseColorTexture.has_value()) {
                            unsigned int baseColorTexture = pbr->baseColorTexture->index;
                            if (baseColorTexture >= 0 && baseColorTexture < _textures.size())
                            {
                                stateSet->setTextureAttributeAndModes(0, _textures[baseColorTexture], osg::StateAttribute::ON);
                            }
                        }
                    }
                }

                applyRenderStyle(geom.get(), primitive);

                geode->addChild(geom);

                osg::ref_ptr<osg::Drawable> edgeDrawable = createStyledEdgeDrawable(geom.get(), primitive);
                if (edgeDrawable.valid())
                {
                    geode->addChild(edgeDrawable.get());
                }

            }
            return geode;
        }

        osg::Drawable* createStyledEdgeDrawable(
            osg::Geometry* geom,
            const CesiumGltf::MeshPrimitive& primitive)
        {
            if (!geom || !_renderStyle || !_renderStyle->enabled || !_renderStyle->strokeEnabled)
            {
                return nullptr;
            }
            if (primitive.mode != CesiumGltf::MeshPrimitive::Mode::TRIANGLES)
            {
                return nullptr;
            }
            if (primitive.indices < 0 || primitive.indices >= static_cast<int>(_arrays.size()))
            {
                return nullptr;
            }

            const osg::Vec3Array* positions = dynamic_cast<const osg::Vec3Array*>(geom->getVertexArray());
            if (!positions || positions->empty())
            {
                return nullptr;
            }

            const osg::Array* primitiveArray = _arrays[primitive.indices].get();
            std::unordered_map<std::uint64_t, EdgeUse> edges;
            if (const osg::UShortArray* indices = dynamic_cast<const osg::UShortArray*>(primitiveArray))
            {
                collectTriangleBoundaryEdges(indices, edges);
            }
            else if (const osg::UIntArray* indices = dynamic_cast<const osg::UIntArray*>(primitiveArray))
            {
                collectTriangleBoundaryEdges(indices, edges);
            }
            else if (const osg::UByteArray* indices = dynamic_cast<const osg::UByteArray*>(primitiveArray))
            {
                collectTriangleBoundaryEdges(indices, edges);
            }

            if (edges.empty())
            {
                return nullptr;
            }

            osg::ref_ptr<osg::Geometry> lines = new osg::Geometry();
            lines->setName(kInlineEdgeDrawableName);
            osg::ref_ptr<osg::Vec3Array> linePositions = new osg::Vec3Array();

            for (const auto& kv : edges)
            {
                const EdgeUse& edge = kv.second;
                if (edge.count != 1u || edge.a >= positions->size() || edge.b >= positions->size())
                {
                    continue;
                }
                const osg::Vec3& a = (*positions)[edge.a];
                const osg::Vec3& b = (*positions)[edge.b];
                if (isTileGridEdge(a, b, _tileFrame))
                {
                    continue;
                }
                linePositions->push_back(a);
                linePositions->push_back(b);
            }

            if (linePositions->empty())
            {
                return nullptr;
            }

            lines->setVertexArray(linePositions.get());
            osg::ref_ptr<osg::Vec4Array> lineColors = new osg::Vec4Array();
            lineColors->push_back(_renderStyle->strokeColor);
            lines->setColorArray(lineColors.get(), osg::Array::BIND_OVERALL);
            lines->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, linePositions->size()));
            lines->setUseDisplayList(false);
            lines->setUseVertexBufferObjects(true);
            osg::StateSet* stateSet = lines->getOrCreateStateSet();
            stateSet->setAttributeAndModes(
                createInlineEdgeProgram(),
                osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
            stateSet->getOrCreateUniform("oe_h3dt_edge_color", osg::Uniform::FLOAT_VEC4)->set(_renderStyle->strokeColor);
            stateSet->getOrCreateUniform("oe_h3dt_edge_width", osg::Uniform::FLOAT)->set(std::max(_renderStyle->strokeWidth, 1.0f));
            stateSet->getOrCreateUniform("oe_h3dt_viewport_size", osg::Uniform::FLOAT_VEC2)->set(osg::Vec2f(1920.0f, 1080.0f));
            stateSet->setAttributeAndModes(
                new osg::LineWidth(std::max(_renderStyle->strokeWidth, 1.0f)),
                osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
            stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
            stateSet->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
            stateSet->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
            stateSet->setAttributeAndModes(
                new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
                osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
            stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            stateSet->setRenderBinDetails(
                _renderStyle->alwaysOnTop ? 100052 : 100051,
                "DepthSortedBin");
            stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
            return lines.release();
        }

        void applyRenderStyle(osg::Geometry* geom, const CesiumGltf::MeshPrimitive& primitive)
        {
            if (!geom)
            {
                return;
            }

            const CesiumGltf::Material* material = nullptr;
            if (primitive.material >= 0 && primitive.material < _model->materials.size())
            {
                material = &_model->materials[primitive.material];
            }

            const bool materialDoubleSided = material && material->doubleSided;
            const bool materialUnlit = material && material->getGenericExtension("KHR_materials_unlit") != nullptr;
            const bool materialBlended = material &&
                (material->alphaMode == CesiumGltf::Material::AlphaMode::BLEND);
            const bool styleEnabled = _renderStyle && _renderStyle->enabled;

            osg::StateSet* stateSet = geom->getOrCreateStateSet();
            if (styleEnabled)
            {
                geom->setName(kInlineMeshDrawableName);
                osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
                colors->push_back(_renderStyle->fillColor);
                geom->setColorArray(colors.get(), osg::Array::BIND_OVERALL);

                osgEarth::MaterialGL3* styleMaterial = new osgEarth::MaterialGL3();
                styleMaterial->setDiffuse(osg::Material::FRONT_AND_BACK, _renderStyle->fillColor);
                styleMaterial->setAmbient(osg::Material::FRONT_AND_BACK, _renderStyle->fillColor);
                stateSet->setAttributeAndModes(
                    styleMaterial,
                    osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
                stateSet->setAttributeAndModes(
                    createInlineMeshProgram(),
                    osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
                stateSet->getOrCreateUniform("oe_h3dt_mesh_color", osg::Uniform::FLOAT_VEC4)->set(_renderStyle->fillColor);
            }

            const bool forceUnlit = styleEnabled && _renderStyle->forceUnlit;
            if (forceUnlit || materialUnlit)
            {
                stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
            }

            const bool forceDoubleSided = styleEnabled && _renderStyle->doubleSided;
            if (forceDoubleSided || materialDoubleSided)
            {
                stateSet->setMode(GL_CULL_FACE, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
            }

            const float alpha = styleEnabled ? _renderStyle->fillColor.a() : 1.f;
            if (materialBlended || alpha < 0.999f)
            {
                stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                stateSet->setMode(GL_BLEND, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
                stateSet->setAttributeAndModes(
                    new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
                    osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
            }

            if (!styleEnabled)
            {
                return;
            }

            if (_renderStyle->alwaysOnTop)
            {
                stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
                stateSet->setRenderBinDetails(100050, "DepthSortedBin");
                stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
            }
            else
            {
                if (_renderStyle->renderBin >= 0)
                {
                    stateSet->setRenderBinDetails(_renderStyle->renderBin, "RenderBin");
                }
                if (_renderStyle->polygonOffset)
                {
                    stateSet->setAttributeAndModes(
                        new osg::PolygonOffset(_renderStyle->polygonOffsetFactor, _renderStyle->polygonOffsetUnits),
                        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
                }
            }
        }

        glm::dmat4 _transform;
        CesiumGltf::Model* _model;
        const TilesetRenderStyleOptions* _renderStyle = nullptr;
        TileLocalFrame _tileFrame;

        std::vector< osg::ref_ptr< osg::Array> > _arrays;
        std::vector< osg::ref_ptr< osg::Texture2D > > _textures;
    };
}
/********/

unsigned int
osgEarth::Cesium::reclampTerrain(
    osg::Node* node,
    const TilesetRenderStyleOptions& renderStyle)
{
    return applyTerrainClampToSubgraph(node, &renderStyle);
}

std::shared_ptr<TilesetTerrainClampTask>
osgEarth::Cesium::snapshotTerrainClampTask(
    osg::Node* node,
    const TilesetRenderStyleOptions& renderStyle)
{
    auto task = std::make_shared<TilesetTerrainClampTask>();
    task->renderStyle = renderStyle;
    if (!node || !renderStyle.clampToGround || !renderStyle.clampMap)
        return task;

    TerrainClampSnapshotVisitor visitor(renderStyle, *task);
    node->accept(visitor);
    return task;
}

bool
osgEarth::Cesium::resolveTerrainClampTask(
    TilesetTerrainClampTask& task,
    osgEarth::Map* map,
    const std::atomic<bool>* cancelFlag)
{
    if (!map || task.geometries.empty())
        return false;

    osg::ref_ptr<const osgEarth::SpatialReference> wgs84 = osgEarth::SpatialReference::get("wgs84");
    if (!wgs84.valid())
        return false;

    osgEarth::Util::ElevationQuery query(map);
    const double desiredResolution =
        task.renderStyle.clampSampleResolutionM > 0.0 ? task.renderStyle.clampSampleResolutionM : 0.0;
    constexpr std::size_t kChunkSize = 2048u;
    bool resolvedAny = false;

    for (TilesetTerrainClampGeometry& geometry : task.geometries)
    {
        if (cancelFlag && cancelFlag->load(std::memory_order_acquire))
            return false;
        if (geometry.samplesLonLatHeight.empty())
            continue;

        bool resolvedGeometry = true;
        for (std::size_t offset = 0; offset < geometry.samplesLonLatHeight.size(); offset += kChunkSize)
        {
            if (cancelFlag && cancelFlag->load(std::memory_order_acquire))
                return false;

            const std::size_t count = std::min(kChunkSize, geometry.samplesLonLatHeight.size() - offset);
            std::vector<osg::Vec3d> chunk(
                geometry.samplesLonLatHeight.begin() + static_cast<std::ptrdiff_t>(offset),
                geometry.samplesLonLatHeight.begin() + static_cast<std::ptrdiff_t>(offset + count));
            if (!query.getElevations(chunk, wgs84.get(), true, desiredResolution))
            {
                resolvedGeometry = false;
                break;
            }
            for (std::size_t i = 0; i < chunk.size(); ++i)
                geometry.samplesLonLatHeight[offset + i].z() = chunk[i].z();
        }

        geometry.resolved = resolvedGeometry;
        resolvedAny = resolvedAny || resolvedGeometry;
    }

    return resolvedAny;
}

unsigned int
osgEarth::Cesium::applyResolvedTerrainClampTask(TilesetTerrainClampTask& task)
{
    osg::ref_ptr<const osgEarth::SpatialReference> wgs84 = osgEarth::SpatialReference::get("wgs84");
    if (!wgs84.valid())
        return 0u;

    unsigned int clampedGeometryCount = 0u;
    for (TilesetTerrainClampGeometry& geometry : task.geometries)
    {
        if (!geometry.resolved || !geometry.geometry.valid() || !geometry.sourcePositions.valid())
            continue;

        osg::ref_ptr<osg::Vec3Array> clampedPositions = new osg::Vec3Array(*geometry.sourcePositions);
        bool wroteAny = false;
        for (std::size_t i = 0; i < clampedPositions->size(); ++i)
        {
            const std::size_t sampleIndex = geometry.vertexToSample[i];
            if (sampleIndex >= geometry.samplesLonLatHeight.size())
                continue;

            const osg::Vec3d& sample = geometry.samplesLonLatHeight[sampleIndex];
            const double terrainHeightM = sample.z();
            if (!terrainHeightSampleValid(terrainHeightM))
                continue;

            osgEarth::GeoPoint clampedPoint(
                wgs84.get(), sample.x(), sample.y(), terrainHeightM, osgEarth::ALTMODE_ABSOLUTE);
            osg::Vec3d clampedEcef;
            if (!clampedPoint.toWorld(clampedEcef))
                continue;

            const osg::Vec3d local = clampedEcef * geometry.ecefToLocal;
            if (!std::isfinite(local.x()) || !std::isfinite(local.y()) || !std::isfinite(local.z()))
                continue;

            (*clampedPositions)[i].set(
                static_cast<float>(local.x()),
                static_cast<float>(local.y()),
                static_cast<float>(local.z()));
            wroteAny = true;
        }

        if (wroteAny)
        {
            geometry.geometry->setVertexArray(clampedPositions.get());
            geometry.geometry->dirtyBound();
            ++clampedGeometryCount;
        }
    }

    return clampedGeometryCount;
}

CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
PrepareRendererResources::prepareInLoadThread(
    const CesiumAsync::AsyncSystem& asyncSystem,
    Cesium3DTilesSelection::TileLoadResult&& tileLoadResult,
    const glm::dmat4& transform,
    const std::any& rendererOptions)
{
    CesiumGltf::Model* model = std::get_if<CesiumGltf::Model>(&tileLoadResult.contentKind);
    if (!model)
    {
        return asyncSystem.createResolvedFuture(
            Cesium3DTilesSelection::TileLoadResultAndRenderResources{
                std::move(tileLoadResult),
                nullptr });
    }


    const TilesetRenderStyleOptions* renderStyle =
        std::any_cast<TilesetRenderStyleOptions>(&rendererOptions);

    TileId tileId;
    TileLocalFrame tileFrame;
    if (tileLoadResult.pCompletedRequest &&
        parseTileIdFromUrl(tileLoadResult.pCompletedRequest->url(), &tileId))
    {
        tileFrame = makeTileLocalFrame(tileId);
    }

    NodeBuilder builder(model, transform, renderStyle, tileFrame);
    LoadThreadResult* result = new LoadThreadResult;
    if (renderStyle)
    {
        result->renderStyle = *renderStyle;
        result->hasRenderStyle = true;
    }
    result->node = builder.build();
    //result->node->setName(tileLoadResult.pCompletedRequest->url());
    return asyncSystem.createResolvedFuture(
        Cesium3DTilesSelection::TileLoadResultAndRenderResources{
            std::move(tileLoadResult),
            result });
}

void* PrepareRendererResources::prepareInMainThread(Cesium3DTilesSelection::Tile& tile, void* pLoadThreadResult)
{    
    LoadThreadResult* loadThreadResult = reinterpret_cast<LoadThreadResult*>(pLoadThreadResult);
    MainThreadResult* mainThreadResult = new MainThreadResult();
    mainThreadResult->node = loadThreadResult->node;

    loadThreadResult->node = nullptr;
    delete loadThreadResult;

    return mainThreadResult;    
}

void PrepareRendererResources::free(
    Cesium3DTilesSelection::Tile& tile,
    void* pLoadThreadResult,
    void* pMainThreadResult) noexcept
{ 
    LoadThreadResult* loadThreadResult = reinterpret_cast<LoadThreadResult*>(pLoadThreadResult);
    if (loadThreadResult)
    {
        if (loadThreadResult->node.valid())
        {
            //result->node->releaseGLObjects();
            loadThreadResult->node = nullptr;
        }
        delete loadThreadResult;
    }
 
    MainThreadResult* mainThreadResult = reinterpret_cast<MainThreadResult*>(pMainThreadResult);
    if (mainThreadResult)
    {
        if (mainThreadResult->node.valid())
        {
            mainThreadResult->node = nullptr;
        }
        delete mainThreadResult;
    }    
}

void* PrepareRendererResources::prepareRasterInLoadThread(
    CesiumGltf::ImageAsset& image,
    const std::any& rendererOptions)
{
    osg::Image* osgImage = getOsgImage(image);

    LoadRasterThreadResult* result = new LoadRasterThreadResult;
    result->image = osgImage;
    return result;
}

void* PrepareRendererResources::prepareRasterInMainThread(
    CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pLoadThreadResult)
{
    LoadRasterThreadResult* loadThreadResult = reinterpret_cast<LoadRasterThreadResult*>(pLoadThreadResult);

    if (!loadThreadResult)
    {
        return nullptr;
    }

    LoadRasterMainThreadResult* result = new LoadRasterMainThreadResult;
    result->image = loadThreadResult->image;
    loadThreadResult->image = nullptr;
    delete loadThreadResult;
    return result;
}

void PrepareRendererResources::freeRaster(
    const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pLoadThreadResult,
    void* pMainThreadResult) noexcept
{
    LoadRasterThreadResult* loadThreadResult = reinterpret_cast<LoadRasterThreadResult*>(pLoadThreadResult);
    if (loadThreadResult)
    {
        loadThreadResult->image = nullptr;
        delete loadThreadResult;
    }

    LoadRasterMainThreadResult* loadMainThreadResult = reinterpret_cast<LoadRasterMainThreadResult*>(pMainThreadResult);
    if (loadMainThreadResult)
    {
        loadMainThreadResult->image = nullptr;
        delete loadMainThreadResult;
    }
}

const char* raster_overlay_vs = R"(
            #version 330
            out vec4 oe_raster_overlay_coords;
            uniform int oe_raster_overlay_coord;
            void raster_overlay_model(inout vec4 vertex)
            {
                switch (oe_raster_overlay_coord) {
                    case 0:
                        oe_raster_overlay_coords = gl_MultiTexCoord0;
                        break;
                    case 1:
                        oe_raster_overlay_coords = gl_MultiTexCoord1;
                        break;
                    case 2:
                        oe_raster_overlay_coords = gl_MultiTexCoord2;
                        break;
                    case 3:
                        oe_raster_overlay_coords = gl_MultiTexCoord3;
                        break;
                }
            }
        )";

const char* raster_overlay_fs = R"(
            #version 330
            in vec4 oe_raster_overlay_coords;
            uniform sampler2D oe_raster_overlay_sampler;
            uniform vec4 oe_translation_scale;
            
            void raster_overlay_color(inout vec4 color)
            {
                vec4 texel;
                vec2 trans = oe_translation_scale.xy;
                vec2 scale = oe_translation_scale.zw;

                vec2 texcoords = oe_raster_overlay_coords.xy * scale + trans;
                texcoords.t = 1.0 - texcoords.t;
                texel = texture(oe_raster_overlay_sampler, texcoords);
                color = color * texel;
            }
        )";

void PrepareRendererResources::attachRasterInMainThread(
    const Cesium3DTilesSelection::Tile& tile,
    int32_t overlayTextureCoordinateID,
    const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pMainThreadRendererResources,
    const glm::dvec2& translation,
    const glm::dvec2& scale)
{
    LoadRasterMainThreadResult* loadMainThreadResult = reinterpret_cast<LoadRasterMainThreadResult*>(pMainThreadRendererResources);
    if (!loadMainThreadResult)
    {
        return;
    }

    const Cesium3DTilesSelection::TileContent& content = tile.getContent();
    const Cesium3DTilesSelection::TileRenderContent* renderContent = content.getRenderContent();

    MainThreadResult* renderResult = reinterpret_cast<MainThreadResult*>(renderContent->getRenderResources());
    if (renderResult)
    {
        int overlayUnit = overlayTextureCoordinateID + 2;
        osg::StateSet* stateset = renderResult->node->getOrCreateStateSet();
        osg::Texture2D* osgTexture = new osg::Texture2D(loadMainThreadResult->image.get());
        osgTexture->setResizeNonPowerOfTwoHint(false);
        osgTexture->setDataVariance(osg::Object::STATIC);
        osgTexture->setFilter(osg::Texture::MIN_FILTER, (osg::Texture::FilterMode)osg::Texture::LINEAR_MIPMAP_LINEAR);
        osgTexture->setFilter(osg::Texture::MAG_FILTER, (osg::Texture::FilterMode)osg::Texture::LINEAR);
        osgTexture->setWrap(osg::Texture::WRAP_S, (osg::Texture::WrapMode)osg::Texture::CLAMP_TO_EDGE);
        osgTexture->setWrap(osg::Texture::WRAP_T, (osg::Texture::WrapMode)osg::Texture::CLAMP_TO_EDGE);
        stateset->setTextureAttributeAndModes(overlayUnit, osgTexture, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);


        stateset->getOrCreateUniform("oe_raster_overlay_coord", osg::Uniform::INT)->set(overlayUnit);        
        stateset->getOrCreateUniform("oe_raster_overlay_sampler", osg::Uniform::SAMPLER_2D)->set(overlayUnit);

        osg::Uniform* uniform_and_scale = new osg::Uniform("oe_translation_scale", osg::Vec4f(translation.x, translation.y, scale.x, scale.y));
        stateset->addUniform(uniform_and_scale);

        VirtualProgram::getOrCreate(stateset)->setFunction("raster_overlay_color", raster_overlay_fs, VirtualProgram::LOCATION_FRAGMENT_COLORING);
        VirtualProgram::getOrCreate(stateset)->setFunction("raster_overlay_model", raster_overlay_vs, VirtualProgram::LOCATION_VERTEX_MODEL);

    }
}

void PrepareRendererResources::detachRasterInMainThread(
    const Cesium3DTilesSelection::Tile& tile,
    int32_t overlayTextureCoordinateID,
    const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pMainThreadRendererResources) noexcept
{
}