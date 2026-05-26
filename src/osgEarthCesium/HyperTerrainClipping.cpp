/* osgEarth Cesium - terrain half-space clipping */
#include "HyperTerrainClipping"

#include <osgEarth/GeoData>
#include <osgEarth/Notify>
#include <osgEarth/SpatialReference>

#include <osg/Uniform>

#include <algorithm>
#include <cmath>

namespace osgEarth {
namespace Cesium {

namespace {

void ensureClippingUniforms(osg::StateSet* ss)
{
    if (!ss)
        return;

    if (!ss->getUniform("hl_clip_enabled"))
        ss->addUniform(new osg::Uniform(osg::Uniform::INT, "hl_clip_enabled"));
    if (!ss->getUniform("hl_clip_union"))
        ss->addUniform(new osg::Uniform(osg::Uniform::INT, "hl_clip_union"));
    if (!ss->getUniform("hl_clip_plane_count"))
        ss->addUniform(new osg::Uniform(osg::Uniform::INT, "hl_clip_plane_count"));
    if (!ss->getUniform("hl_clip_modelMatrixInverse"))
        ss->addUniform(new osg::Uniform(osg::Uniform::FLOAT_MAT4, "hl_clip_modelMatrixInverse"));
    if (!ss->getUniform("hl_clip_planes")) {
        auto* planesU = new osg::Uniform(
            osg::Uniform::FLOAT_VEC4,
            "hl_clip_planes",
            kHyperTerrainMaxClippingPlanes);
        for (int i = 0; i < kHyperTerrainMaxClippingPlanes; ++i)
            planesU->setElement(i, osg::Vec4f(0.f, 0.f, 0.f, 0.f));
        ss->addUniform(planesU);
    }
}

bool tryBuildEdgePlaneFromEcef(
    const osg::Vec3d& p0,
    const osg::Vec3d& p1,
    HyperTerrainClippingPlane& out)
{
    osg::Vec3d midpoint = (p0 + p1) * 0.5;
    const double midLen = midpoint.length();
    if (midLen < 1e-6)
        return false;

    osg::Vec3d up = midpoint / midLen;

    osg::Vec3d right = p1 - midpoint;
    const double rightLen = right.length();
    if (rightLen < 1e-6)
        return false;
    right /= rightLen;

    osg::Vec3d normal = right ^ up;
    const double nLen = normal.length();
    if (nLen < 1e-6)
        return false;
    normal /= nLen;

    const double distance = normal * midpoint;

    out.normal.set(
        static_cast<float>(normal.x()),
        static_cast<float>(normal.y()),
        static_cast<float>(normal.z()));
    out.distance = static_cast<float>(distance);
    return true;
}

} // namespace

void initHyperTerrainClippingUniforms(osg::StateSet* ss)
{
    ensureClippingUniforms(ss);
    applyHyperTerrainClippingToStateSet(ss, HyperTerrainClippingState{});
}

void applyHyperTerrainClippingToStateSet(
    osg::StateSet* ss,
    const HyperTerrainClippingState& state)
{
    if (!ss)
        return;

    ensureClippingUniforms(ss);

    const bool active = state.enabled && !state.planes.empty();
    ss->getUniform("hl_clip_enabled")->set(active ? 1 : 0);
    ss->getUniform("hl_clip_union")->set(state.unionClippingRegions ? 1 : 0);

    const int count = static_cast<int>(std::min(
        state.planes.size(),
        static_cast<size_t>(kHyperTerrainMaxClippingPlanes)));
    ss->getUniform("hl_clip_plane_count")->set(count);

    osg::Matrixd inv = state.modelMatrix;
    if (!inv.invert(inv))
        inv.makeIdentity();
    ss->getUniform("hl_clip_modelMatrixInverse")->set(osg::Matrixf(inv));

    osg::Uniform* planesU = ss->getUniform("hl_clip_planes");
    for (int i = 0; i < kHyperTerrainMaxClippingPlanes; ++i) {
        if (i < count) {
            const HyperTerrainClippingPlane& p = state.planes[static_cast<size_t>(i)];
            planesU->setElement(i, osg::Vec4f(p.normal.x(), p.normal.y(), p.normal.z(), p.distance));
        } else {
            planesU->setElement(i, osg::Vec4f(0.f, 0.f, 0.f, 0.f));
        }
    }
}

HyperTerrainClippingState makePolygonClippingStateFromEcef(
    const std::vector<osg::Vec3d>& ringEcef)
{
    HyperTerrainClippingState state;
    state.modelMatrix.makeIdentity();
    state.unionClippingRegions = false;

    if (ringEcef.size() < 3) {
        state.enabled = false;
        return state;
    }

    const size_t n = ringEcef.size();
    state.planes.reserve(std::min(n, static_cast<size_t>(kHyperTerrainMaxClippingPlanes)));

    for (size_t i = 0; i < n; ++i) {
        if (state.planes.size() >= static_cast<size_t>(kHyperTerrainMaxClippingPlanes)) {
            OSG_WARN << "[HyperTerrainClipping] Polygon has " << n
                     << " edges; only first " << kHyperTerrainMaxClippingPlanes
                     << " clipping planes are applied." << std::endl;
            break;
        }
        const size_t next = (i + 1) % n;
        HyperTerrainClippingPlane plane;
        if (tryBuildEdgePlaneFromEcef(ringEcef[i], ringEcef[next], plane))
            state.planes.push_back(plane);
    }

    state.enabled = state.planes.size() >= 3;
    return state;
}

HyperTerrainClippingState makePolygonClippingStateFromGeo(
    const std::vector<HyperTerrainClippingLonLatHeight>& ringLonLatHeight)
{
    std::vector<osg::Vec3d> ringEcef;
    ringEcef.reserve(ringLonLatHeight.size());

    const osgEarth::SpatialReference* wgs84 = osgEarth::SpatialReference::get("wgs84");
    for (const HyperTerrainClippingLonLatHeight& llh : ringLonLatHeight) {
        osgEarth::GeoPoint gp(
            wgs84,
            llh.x(),
            llh.y(),
            llh.z(),
            osgEarth::ALTMODE_ABSOLUTE);
        osg::Vec3d world;
        gp.toWorld(world);
        ringEcef.push_back(world);
    }

    return makePolygonClippingStateFromEcef(ringEcef);
}

osg::Matrixd makeEcefEnuModelMatrix(double lonDeg, double latDeg, double heightM)
{
    const osgEarth::SpatialReference* wgs84 = osgEarth::SpatialReference::get("wgs84");
    osgEarth::GeoPoint anchor(wgs84, lonDeg, latDeg, heightM, osgEarth::ALTMODE_ABSOLUTE);
    osg::Matrixd localToWorld;
    if (!anchor.createLocalToWorld(localToWorld))
        localToWorld.makeIdentity();
    return localToWorld;
}

} // namespace Cesium
} // namespace osgEarth
