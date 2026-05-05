/* osgEarth
 * Copyright 2025 Pelican Mapping
 * MIT License
 */
#include "QuantizedMeshElevationLayer"
#include "HeightFieldUtils"
#include "Map"
#include <osgEarth/SpatialReference>
#include <limits>
#include <cmath>

using namespace osgEarth;

#define LC "[QuantizedMeshElevationLayer] "

REGISTER_OSGEARTH_LAYER(quantizedmeshelevation, QuantizedMeshElevationLayer);

//...........................................................................

void
QuantizedMeshElevationLayer::Options::fromConfig(const Config& /*conf*/)
{
    // no custom options beyond base ElevationLayer
}

Config
QuantizedMeshElevationLayer::Options::getConfig() const
{
    return ElevationLayer::Options::getConfig();
}

//...........................................................................

Status
QuantizedMeshElevationLayer::openImplementation()
{
    OE_RETURN_STATUS_ON_ERROR(super::openImplementation());

    if (!_source.valid())
    {
        return Status(Status::ResourceUnavailable,
                      "QuantizedMeshElevationLayer requires a source TerrainMeshLayer");
    }

    if (!_source->isOpen())
    {
        const Status s = _source->open(getReadOptions());
        if (s.isError())
            return Status(Status::ResourceUnavailable,
                          "Source TerrainMeshLayer failed to open: " + s.message());
    }

    return Status::NoError;
}

void
QuantizedMeshElevationLayer::addedToMap(const Map* map)
{
    super::addedToMap(map);
    _map = map;

    if (_source.valid() && _source->isOpen())
    {
        _source->addedToMap(map);

        // Mirror the source's data extents so ElevationPool queries are scoped correctly.
        DataExtentList extents;
        _source->getDataExtents(extents);
        if (!extents.empty())
            setDataExtents(extents);
    }
}

void
QuantizedMeshElevationLayer::removedFromMap(const Map* map)
{
    if (_source.valid() && _source->isOpen())
        _source->removedFromMap(map);
    super::removedFromMap(map);
}

GeoHeightField
QuantizedMeshElevationLayer::createHeightFieldImplementation(
    const TileKey& key,
    ProgressCallback* progress) const
{
    if (!_source.valid() || !_source->isOpen())
        return GeoHeightField::INVALID;

    // Upsampling: if the requested LOD is beyond the source's actual data level,
    // fetch the nearest ancestor that has data and bilinearly subsample it to
    // the requested extent. This produces a finer terrain mesh (better UV mapping)
    // without requiring additional tile data on the server.
    if (_maxSourceLevel < 99u && key.getLOD() > _maxSourceLevel)
    {
        TileKey ancestorKey = key.createAncestorKey(static_cast<int>(_maxSourceLevel));
        if (!ancestorKey.valid())
            return GeoHeightField::INVALID;

        GeoHeightField ancestorHF = createHeightFieldImplementation(ancestorKey, progress);
        if (!ancestorHF.valid())
            return GeoHeightField::INVALID;

        return ancestorHF.createSubSample(
            key.getExtent(), getTileSize(), getTileSize(), INTERP_BILINEAR);
    }

    // Fetch the terrain mesh in local (LTP) space.
    TileMesh mesh = _source->createTile(key, progress);
    if (!mesh.verts.valid() || mesh.verts->empty())
        return GeoHeightField::INVALID;

    // WGS84 geographic SRS for ECEF → (lon, lat, altHAE) conversion.
    const osgEarth::SpatialReference* wgs84 =
        osgEarth::SpatialReference::get("wgs84");
    if (!wgs84)
        return GeoHeightField::INVALID;

    // Build a list of (u, v, altHAE) samples from TileMesh verts.
    // verts are in local (LTP) space; apply localToWorld to get ECEF.
    const std::size_t numVerts = mesh.verts->size();
    const bool hasUvs = mesh.uvs.valid() && mesh.uvs->size() == numVerts;

    struct Sample { float u, v, altM; };
    std::vector<Sample> samples;
    samples.reserve(numVerts);

    for (std::size_t i = 0; i < numVerts; ++i)
    {
        const osg::Vec3& local = (*mesh.verts)[i];
        // Transform vert from local (LTP) space to ECEF world space.
        const osg::Vec3d ecef =
            osg::Vec3d(local.x(), local.y(), local.z()) * mesh.localToWorld;

        osg::Vec3d lla;
        if (!wgs84->transformFromWorld(ecef, lla))
            continue;

        float u = 0.0f, v = 0.0f;
        if (hasUvs)
        {
            u = (*mesh.uvs)[i].x();
            v = (*mesh.uvs)[i].y();
        }
        else
        {
            // Approximate UV from geographic position within the tile extent.
            const GeoExtent& ex = key.getExtent();
            u = static_cast<float>((lla.x() - ex.xMin()) / ex.width());
            v = static_cast<float>((lla.y() - ex.yMin()) / ex.height());
        }

        samples.push_back({u, v, static_cast<float>(lla.z())});
    }

    if (samples.empty())
        return GeoHeightField::INVALID;

    // Create HeightField grid and fill by nearest-UV sample.
    const unsigned tileSize = getTileSize();

    auto hf = HeightFieldUtils::createReferenceHeightField(
        key.getExtent(), tileSize, tileSize, 0u, false, static_cast<float>(NO_DATA_VALUE));

    if (!hf.valid())
        return GeoHeightField::INVALID;

    bool anyFilled = false;

    for (unsigned row = 0; row < tileSize; ++row)
    {
        const float v = static_cast<float>(row) / static_cast<float>(tileSize - 1u);
        for (unsigned col = 0; col < tileSize; ++col)
        {
            const float u = static_cast<float>(col) / static_cast<float>(tileSize - 1u);

            float bestDist2 = std::numeric_limits<float>::max();
            float bestAlt = static_cast<float>(NO_DATA_VALUE);

            for (const auto& s : samples)
            {
                const float du = s.u - u;
                const float dv = s.v - v;
                const float d2 = du * du + dv * dv;
                if (d2 < bestDist2)
                {
                    bestDist2 = d2;
                    bestAlt = s.altM;
                }
            }

            if (bestAlt != static_cast<float>(NO_DATA_VALUE))
            {
                hf->setHeight(col, row, bestAlt);
                anyFilled = true;
            }
        }
    }

    if (!anyFilled)
        return GeoHeightField::INVALID;

    return GeoHeightField(hf.release(), key.getExtent());
}
