/* osgEarth
* Copyright 2008-2014 Pelican Mapping
* MIT License
*/
#include "SelectionInfo"
#include <osgEarth/TileKey>

using namespace osgEarth::REX;
using namespace osgEarth;

#define LC "[SelectionInfo] "

const double SelectionInfo::_morphStartRatio = 0.66;

bool
SelectionInfo::computeLODTable(unsigned firstLod, unsigned maxLod, const Profile* profile,
    double mtrf, bool restrictPolarSubdivision,
    const std::function<double(double, double)>& mapMTRF,
    std::vector<LOD>& out_lods)
{
    OE_SOFT_ASSERT_AND_RETURN(profile != nullptr, false);
    OE_SOFT_ASSERT_AND_RETURN(profile->getSRS() != nullptr && profile->getSRS()->valid(), false);

    if (firstLod > maxLod)
    {
        OE_WARN << LC << "Error: Inconsistent First and Max LODs" << std::endl;
        return false;
    }

    unsigned numLods = maxLod + 1u;

    out_lods.resize(numLods);

    for (unsigned lod = 0; lod <= maxLod; ++lod)
    {
        unsigned tx, ty;
        profile->getNumTiles(lod, tx, ty);
        TileKey key(lod, tx / 2, ty / 2, profile);
        GeoExtent e = key.getExtent();
        GeoCircle c = e.computeBoundingGeoCircle();
        double range = c.getRadius() * mtrf * 2.0 * (1.0 / 1.405);
        range = c.getRadius() * mapMTRF(mtrf, range) * 2.0 * (1.0 / 1.405);
        out_lods[lod]._visibilityRange = range;
        out_lods[lod]._minValidTY = 0;
        out_lods[lod]._maxValidTY = 0xFFFFFFFF;
    }

    double prevPos = 0.0;

    for (int lod = (int)(numLods - 1); lod >= 0; --lod)
    {
        double span = out_lods[lod]._visibilityRange - prevPos;

        out_lods[lod]._morphEnd = out_lods[lod]._visibilityRange;
        out_lods[lod]._morphStart = prevPos + span * _morphStartRatio;
        prevPos = out_lods[lod]._morphEnd;

        int startLOD = 6;
        if (restrictPolarSubdivision && lod >= startLOD && profile->getSRS()->isGeographic())
        {
            const double startAR = 0.1;
            const double endAR = 0.4;
            double lodT = (double)(lod - startLOD) / (double)(numLods - 1);
            double minAR = startAR + (endAR - startAR) * lodT;

            unsigned tx, ty;
            profile->getNumTiles(lod, tx, ty);
            for (int y = (int)ty / 2; y >= 0; --y)
            {
                TileKey k(lod, 0, y, profile);
                const GeoExtent& e = k.getExtent();
                double width_m = e.width(Units::METERS);
                double height_m = e.height(Units::METERS);
                if (width_m / height_m < minAR)
                {
                    out_lods[lod]._minValidTY = std::min(y + 1, (int)(ty - 1));
                    out_lods[lod]._maxValidTY = (ty - 1) - out_lods[lod]._minValidTY;
                    break;
                }
            }
        }
    }

    return true;
}

void
SelectionInfo::commitUnlocked(unsigned firstLod, unsigned maxLod, osg::ref_ptr<const Profile> profile,
    double mtrf, bool restrictPolarSubdivision, std::vector<LOD>&& lods)
{
    _firstLOD = firstLod;
    _maxLOD = maxLod;
    _profile = profile;
    _mrtf = mtrf;
    _restrictPolarSubdivision = restrictPolarSubdivision;
    _lods = std::move(lods);
}

void
SelectionInfo::commitLODTable(unsigned firstLod, unsigned maxLod, osg::ref_ptr<const Profile> profile,
    double mtrf, bool restrictPolarSubdivision, std::vector<LOD>&& lods)
{
    std::unique_lock<std::shared_mutex> lock(_mutex);
    commitUnlocked(firstLod, maxLod, profile, mtrf, restrictPolarSubdivision, std::move(lods));
}

void
SelectionInfo::initialize(unsigned firstLod, unsigned maxLod, const Profile* profile, double mtrf, bool restrictPolarSubdivision)
{
    std::function<double(double, double)> mapMTRF_copy;
    {
        std::shared_lock<std::shared_mutex> lk(_mutex);
        mapMTRF_copy = _mapMTRF;
    }

    std::vector<LOD> lods;
    if (!computeLODTable(firstLod, maxLod, profile, mtrf, restrictPolarSubdivision, mapMTRF_copy, lods))
        return;

    std::unique_lock<std::shared_mutex> lock(_mutex);
    commitUnlocked(firstLod, maxLod, profile, mtrf, restrictPolarSubdivision, std::move(lods));
}

void
SelectionInfo::initializePlaceholder(unsigned firstLod, unsigned maxLod, const Profile* profile, double mtrf, bool restrictPolarSubdivision)
{
    std::function<double(double, double)> mapMTRF_copy;
    {
        std::shared_lock<std::shared_mutex> lk(_mutex);
        mapMTRF_copy = _mapMTRF;
    }

    std::vector<LOD> lods;
    if (!computeLODTable(firstLod, maxLod, profile, mtrf, false, mapMTRF_copy, lods))
        return;

    std::unique_lock<std::shared_mutex> lock(_mutex);
    commitUnlocked(firstLod, maxLod, profile, mtrf, restrictPolarSubdivision, std::move(lods));
}

unsigned
SelectionInfo::getNumLODs(void) const
{
    std::shared_lock<std::shared_mutex> lk(_mutex);
    return (unsigned)_lods.size();
}

const SelectionInfo::LOD&
SelectionInfo::getLOD(unsigned lod) const
{
    static SelectionInfo::LOD s_dummy;

    std::shared_lock<std::shared_mutex> lk(_mutex);

    if (lod - _firstLOD >= _lods.size())
    {
        return s_dummy;
    }
    return _lods[lod - _firstLOD];
}

float
SelectionInfo::getRange(const TileKey& key) const
{
    std::shared_lock<std::shared_mutex> lk(_mutex);

    if (key.getLOD() >= _lods.size())
        return 0.0f;

    const LOD& lod = _lods[key.getLOD()];
    if (key.getTileY() >= lod._minValidTY && key.getTileY() <= lod._maxValidTY)
    {
        return (float)lod._visibilityRange;
    }
    return 0.0f;
}

void
SelectionInfo::setMTRFMappingFunction(std::function<double(double, double)> func)
{
    unsigned fl;
    unsigned ml;
    osg::ref_ptr<const Profile> prof;
    double mrtf;
    bool rps;

    {
        std::unique_lock<std::shared_mutex> lock(_mutex);
        _mapMTRF = func;
        fl = _firstLOD;
        ml = _maxLOD;
        prof = _profile;
        mrtf = _mrtf;
        rps = _restrictPolarSubdivision;
    }

    if (!prof.valid())
        return;

    std::vector<LOD> lods;
    if (!computeLODTable(fl, ml, prof.get(), mrtf, rps, func, lods))
        return;

    std::unique_lock<std::shared_mutex> lock(_mutex);
    _lods = std::move(lods);
}

void
SelectionInfo::get(const TileKey& key,
    float& out_range,
    float& out_startMorphRange,
    float& out_endMorphRange) const
{
    out_range = 0.0f;
    out_startMorphRange = 0.0f;
    out_endMorphRange = 0.0f;

    std::shared_lock<std::shared_mutex> lk(_mutex);

    if (key.getLOD() < _lods.size())
    {
        const LOD& lod = _lods[key.getLOD()];

        if (key.getTileY() >= lod._minValidTY && key.getTileY() <= lod._maxValidTY)
        {
            out_range = (float)lod._visibilityRange;
            out_startMorphRange = (float)lod._morphStart;
            out_endMorphRange = (float)lod._morphEnd;
        }
    }
}
