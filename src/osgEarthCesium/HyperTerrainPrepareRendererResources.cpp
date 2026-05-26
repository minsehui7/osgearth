// WINGDIAPI/APIENTRY and std::result_of compat provided by ForcedInclude compat_result_of.h

#include "HyperTerrainPrepareRendererResources"
#include "HyperTerrainLitShader"
#include "HyperTerrainImageryFactory"
#include "HyperTerrainRasterRenderer"

#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/TileContent.h>
#include <Cesium3DTilesSelection/TileLoadResult.h>
#include <Cesium3DTilesSelection/RasterMappedTo3DTile.h>
#include <CesiumAsync/AsyncSystem.h>
#include <CesiumGltf/Model.h>
#include <CesiumGltf/Accessor.h>
#include <CesiumGltf/MeshPrimitive.h>
#include <CesiumGltf/AccessorView.h>
#include <CesiumRasterOverlays/RasterOverlayTile.h>

#include <algorithm>
#include <cmath>

#include <osg/BlendFunc>
#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osg/Texture2D>
#include <osg/Notify>
#include <osg/Program>
#include <osg/Shader>
#include <osg/RenderInfo>

#include <osgEarth/Lighting>

#include <glm/mat4x4.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/geometric.hpp>

namespace osgEarth { namespace Cesium {

namespace {

static constexpr int kMaxOverlays = 4;

// prepareInLoadThread → prepareInMainThread 데이터
struct LoadThreadData {
    glm::dmat4 transform; // tile local → ECEF 변환 (prepareInLoadThread에서 쫭처)
};

// glTF NORMAL accessor 추출 또는 삼각형에서 면적 가중 법선 계산
osg::Vec3Array* buildNormals(
    const CesiumGltf::Model& model,
    const CesiumGltf::MeshPrimitive& prim,
    const CesiumGltf::AccessorView<glm::vec3>& positions)
{
    const size_t nVerts = static_cast<size_t>(positions.size());

    auto normIt = prim.attributes.find("NORMAL");
    if (normIt != prim.attributes.end() && normIt->second >= 0
        && static_cast<size_t>(normIt->second) < model.accessors.size()) {
        CesiumGltf::AccessorView<glm::vec3> normView(
            model, model.accessors[static_cast<size_t>(normIt->second)]);
        if (normView.status() == CesiumGltf::AccessorViewStatus::Valid
            && static_cast<size_t>(normView.size()) == nVerts) {
            auto* arr = new osg::Vec3Array(nVerts);
            for (size_t i = 0; i < nVerts; ++i)
                (*arr)[i].set(normView[static_cast<int64_t>(i)].x,
                              normView[static_cast<int64_t>(i)].y,
                              normView[static_cast<int64_t>(i)].z);
            return arr;
        }
    }

    std::vector<glm::vec3> normals(nVerts, glm::vec3(0.0f));
    auto accumTri = [&](size_t i0, size_t i1, size_t i2) {
        const glm::vec3 v0(positions[static_cast<int64_t>(i0)].x,
                           positions[static_cast<int64_t>(i0)].y,
                           positions[static_cast<int64_t>(i0)].z);
        const glm::vec3 v1(positions[static_cast<int64_t>(i1)].x,
                           positions[static_cast<int64_t>(i1)].y,
                           positions[static_cast<int64_t>(i1)].z);
        const glm::vec3 v2(positions[static_cast<int64_t>(i2)].x,
                           positions[static_cast<int64_t>(i2)].y,
                           positions[static_cast<int64_t>(i2)].z);
        const glm::vec3 n = glm::cross(v1 - v0, v2 - v0);
        normals[i0] += n;
        normals[i1] += n;
        normals[i2] += n;
    };

    if (prim.indices >= 0
        && static_cast<size_t>(prim.indices) < model.accessors.size()) {
        const auto& idxAcc = model.accessors[static_cast<size_t>(prim.indices)];
        CesiumGltf::AccessorView<uint16_t> idx16(model, idxAcc);
        if (idx16.status() == CesiumGltf::AccessorViewStatus::Valid) {
            for (int64_t i = 0; i + 2 < idx16.size(); i += 3)
                accumTri(idx16[i], idx16[i + 1], idx16[i + 2]);
        } else {
            CesiumGltf::AccessorView<uint32_t> idx32(model, idxAcc);
            if (idx32.status() == CesiumGltf::AccessorViewStatus::Valid)
                for (int64_t i = 0; i + 2 < idx32.size(); i += 3)
                    accumTri(idx32[i], idx32[i + 1], idx32[i + 2]);
        }
    } else {
        for (size_t i = 0; i + 2 < nVerts; i += 3)
            accumTri(i, i + 1, i + 2);
    }

    auto* arr = new osg::Vec3Array(nVerts);
    for (size_t i = 0; i < nVerts; ++i) {
        const float len = glm::length(normals[i]);
        (*arr)[i] = (len > 1e-6f)
            ? osg::Vec3f(normals[i].x / len, normals[i].y / len, normals[i].z / len)
            : osg::Vec3f(0.0f, 1.0f, 0.0f);
    }
    return arr;
}

} // anonymous namespace

HyperTerrainPrepareRendererResources::HyperTerrainPrepareRendererResources(osg::Group* sceneRoot)
    : m_sceneRoot(sceneRoot)
{}

HyperTerrainTileRenderHandle* HyperTerrainPrepareRendererResources::allocateHandle(const HyperTerrainTileRenderData& data) {
    std::lock_guard<std::mutex> lock(m_storeMutex);
    ++m_tickCounter;

    uint32_t slot = 0;
    if (!m_freeSlots.empty()) {
        slot = m_freeSlots.back();
        m_freeSlots.pop_back();
    } else {
        slot = static_cast<uint32_t>(m_store.size());
        m_store.emplace_back();
    }

    TileRenderRecord& record = m_store[slot];
    record.data = data;
    record.alive = true;
    record.createdTick = m_tickCounter;
    record.lastAccessTick = m_tickCounter;

    auto handleStorage = std::make_unique<HyperTerrainTileRenderHandle>();
    HyperTerrainTileRenderHandle* handle = handleStorage.get();
    handle->slot = slot;
    handle->generation = record.generation;
    m_handleStorage.push_back(std::move(handleStorage));
    return handle;
}

bool HyperTerrainPrepareRendererResources::resolveRenderResources(
    const void* renderResources,
    HyperTerrainTileRenderData& outData) const {
    return resolveHandle(static_cast<const HyperTerrainTileRenderHandle*>(renderResources), outData);
}

bool HyperTerrainPrepareRendererResources::resolveHandle(
    const HyperTerrainTileRenderHandle* handle,
    HyperTerrainTileRenderData& outData) const {
    if (!handle) return false;

    std::lock_guard<std::mutex> lock(m_storeMutex);
    if (handle->slot >= m_store.size()) return false;

    const TileRenderRecord& record = m_store[handle->slot];
    if (!record.alive || record.generation != handle->generation) {
        if (m_invalidHandleLogs < 16) {
            ++m_invalidHandleLogs;
            OSG_WARN << "[HyperTerrain] Invalid HyperTerrainTileRenderHandle:"
                     << " slot=" << handle->slot
                     << " gen=" << handle->generation
                     << " recordGen=" << record.generation
                     << " alive=" << record.alive
                     << std::endl;
        }
        return false;
    }

    outData = record.data;
    m_store[handle->slot].lastAccessTick = ++m_tickCounter;
    return true;
}

void HyperTerrainPrepareRendererResources::releaseHandle(const HyperTerrainTileRenderHandle* handle) {
    if (!handle) return;

    std::lock_guard<std::mutex> lock(m_storeMutex);
    if (handle->slot >= m_store.size()) return;

    TileRenderRecord& record = m_store[handle->slot];
    if (!record.alive || record.generation != handle->generation) return;

    record.alive = false;
    record.data = HyperTerrainTileRenderData{};
    ++record.generation;
    m_freeSlots.push_back(handle->slot);
}

bool HyperTerrainPrepareRendererResources::tryResolveTileRenderData(
    const Cesium3DTilesSelection::Tile& tile,
    HyperTerrainTileRenderData& outData) const {
    const auto* rc = tile.getContent().getRenderContent();
    if (!rc) return false;
    return resolveRenderResources(rc->getRenderResources(), outData);
}

float HyperTerrainPrepareRendererResources::rasterOverlayAlpha(
    const CesiumRasterOverlays::RasterOverlay* overlay) const
{
    if (!overlay) return 1.0f;
    std::lock_guard<std::mutex> lock(m_overlayMutex);
    const auto it = m_overlayAlpha.find(overlay);
    return it != m_overlayAlpha.end() ? it->second : 1.0f;
}

void HyperTerrainPrepareRendererResources::setRasterOverlayAlpha(
    const CesiumRasterOverlays::RasterOverlay* overlay,
    float alpha)
{
    if (!overlay) return;

    alpha = std::clamp(alpha, 0.0f, 1.0f);
    std::lock_guard<std::mutex> lock(m_overlayMutex);
    m_overlayAlpha[overlay] = alpha;

    for (RasterAttachment& attachment : m_rasterAttachments) {
        if (attachment.overlay != overlay || !attachment.stateSet.valid() || attachment.slot < 0)
            continue;
        if (auto* u = attachment.stateSet->getUniform("u_overlayAlpha"))
            u->setElement(attachment.slot, alpha);
    }
}

void HyperTerrainPrepareRendererResources::setTmsImageryExposure(float exposure)
{
    exposure = std::max(0.0f, exposure);
    HyperTerrainImageryFactory::setTmsImageryExposure(exposure);

    std::lock_guard<std::mutex> lock(m_storeMutex);
    for (TileRenderRecord& record : m_store) {
        if (!record.alive || !record.data.geom.valid())
            continue;
        osg::StateSet* const ss = record.data.geom->getStateSet();
        if (!ss)
            continue;
        if (osg::Uniform* u = ss->getUniform("u_tmsImageryExposure"))
            u->set(exposure);
        else
            ss->getOrCreateUniform("u_tmsImageryExposure", osg::Uniform::FLOAT)->set(exposure);
    }
}

CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
HyperTerrainPrepareRendererResources::prepareInLoadThread(
    const CesiumAsync::AsyncSystem& asyncSystem,
    Cesium3DTilesSelection::TileLoadResult&& tileLoadResult,
    const glm::dmat4& transform,
    const std::any& /*rendererOptions*/)
{
    // 워커 스레드: transform을 쫭처해 main thread에 전달
    auto* ltData = new LoadThreadData();
    ltData->transform = transform;
    return asyncSystem.createResolvedFuture(
        Cesium3DTilesSelection::TileLoadResultAndRenderResources{
            std::move(tileLoadResult), ltData });
}

void* HyperTerrainPrepareRendererResources::prepareInMainThread(
    Cesium3DTilesSelection::Tile& tile,
    void* pLoadThreadResult)
{
    const auto* pContent = tile.getContent().getRenderContent();
    if (!pContent) {
        OSG_WARN << "[HyperTerrain] prepareInMainThread: no render content!" << std::endl;
        if (pLoadThreadResult) delete static_cast<LoadThreadData*>(pLoadThreadResult);
        return nullptr;
    }

    const CesiumGltf::Model& model = pContent->getModel();

    // ---- POSITION → Vec3Array ----
    if (model.meshes.empty() || model.meshes[0].primitives.empty()) {
        if (pLoadThreadResult) delete static_cast<LoadThreadData*>(pLoadThreadResult);
        return nullptr;
    }
    const auto& prim = model.meshes[0].primitives[0];
    auto posIt = prim.attributes.find("POSITION");
    if (posIt == prim.attributes.end() || posIt->second < 0) {
        if (pLoadThreadResult) delete static_cast<LoadThreadData*>(pLoadThreadResult);
        return nullptr;
    }
    const auto& posAcc = model.accessors[static_cast<size_t>(posIt->second)];
    CesiumGltf::AccessorView<glm::vec3> positions(model, posAcc);
    if (positions.status() != CesiumGltf::AccessorViewStatus::Valid || positions.size() == 0) {
        if (pLoadThreadResult) delete static_cast<LoadThreadData*>(pLoadThreadResult);
        return nullptr;
    }

    auto* vtx = new osg::Vec3Array(static_cast<size_t>(positions.size()));
    for (int64_t i = 0; i < positions.size(); ++i)
        (*vtx)[static_cast<size_t>(i)].set(positions[i].x, positions[i].y, positions[i].z);

    // ---- 법선 계산 (glTF NORMAL accessor 또는 삼각형에서 계산) ----
    osg::Vec3Array* nrm = buildNormals(model, prim, positions);

    // ---- 인덱스 버퍼 ----
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    geom->setUseDisplayList(false);
    geom->setUseVertexBufferObjects(true);
    geom->setVertexAttribArray(0, vtx, osg::Array::BIND_PER_VERTEX);
    geom->setVertexAttribArray(1, nrm, osg::Array::BIND_PER_VERTEX);

    // ---- _CESIUMOVERLAY_0..3 for raster overlay UV (attribute location 2..5) ----
    // cesium-native는 오버레이 UV를 TEXCOORD_N이 아닌 _CESIUMOVERLAY_N 이름으로 저장한다.
    for (int tc = 0; tc < kMaxOverlays; ++tc) {
        const std::string name = std::string("_CESIUMOVERLAY_") + std::to_string(tc);
        auto tcIt = prim.attributes.find(name);
        if (tcIt != prim.attributes.end() && tcIt->second >= 0
            && static_cast<size_t>(tcIt->second) < model.accessors.size()) {
            CesiumGltf::AccessorView<glm::vec2> tcView(
                model, model.accessors[static_cast<size_t>(tcIt->second)]);
            if (tcView.status() == CesiumGltf::AccessorViewStatus::Valid && tcView.size() > 0) {
                auto* tcArr = new osg::Vec2Array(static_cast<size_t>(tcView.size()));
                for (int64_t i = 0; i < tcView.size(); ++i)
                    (*tcArr)[static_cast<size_t>(i)].set(tcView[i].x, tcView[i].y);
                geom->setVertexAttribArray(2 + tc, tcArr, osg::Array::BIND_PER_VERTEX);
            }
        }
    }

    // 동일 투영(Web Mercator 등)의 오버레이는 glTF에 _CESIUMOVERLAY_0 만 있고 ID는 둘 다 0이다.
    // 두 번째 오버레이를 OSG 유닛 1..3에 올릴 때 동일 Mercator UV가 필요하므로, 첫 유효 슬롯 배열을 공유한다.
    {
        osg::Vec2Array* sharedMercatorUvs = nullptr;
        for (int tc = 0; tc < kMaxOverlays; ++tc) {
            osg::Array* arr = geom->getVertexAttribArray(2 + tc);
            if (arr) {
                sharedMercatorUvs = dynamic_cast<osg::Vec2Array*>(arr);
                if (sharedMercatorUvs)
                    break;
            }
        }
        if (sharedMercatorUvs) {
            for (int tc = 0; tc < kMaxOverlays; ++tc) {
                if (!geom->getVertexAttribArray(2 + tc)) {
                    geom->setVertexAttribArray(2 + tc, sharedMercatorUvs, osg::Array::BIND_PER_VERTEX);
                }
            }
        }
    }

    if (prim.indices >= 0 && static_cast<size_t>(prim.indices) < model.accessors.size()) {
        const auto& idxAcc = model.accessors[static_cast<size_t>(prim.indices)];
        CesiumGltf::AccessorView<uint16_t> idx16(model, idxAcc);
        if (idx16.status() == CesiumGltf::AccessorViewStatus::Valid) {
            auto* de = new osg::DrawElementsUShort(osg::PrimitiveSet::TRIANGLES, static_cast<size_t>(idx16.size()));
            for (int64_t i = 0; i < idx16.size(); ++i) (*de)[static_cast<size_t>(i)] = idx16[i];
            geom->addPrimitiveSet(de);
        } else {
            CesiumGltf::AccessorView<uint32_t> idx32(model, idxAcc);
            if (idx32.status() == CesiumGltf::AccessorViewStatus::Valid) {
                auto* de = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, static_cast<size_t>(idx32.size()));
                for (int64_t i = 0; i < idx32.size(); ++i) (*de)[static_cast<size_t>(i)] = idx32[i];
                geom->addPrimitiveSet(de);
            }
        }
    } else {
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, static_cast<int>(positions.size())));
    }

    // ---- 셰이더 설정 ----
    osg::StateSet* ss = geom->getOrCreateStateSet();
    // Keep terrain material local so clamping RTT/depth passes can still install
    // their own state as needed.
    ss->setAttribute(
        createHyperTerrainLitProgram(HyperTerrainLitColorMode::FixedTerrainBase),
        osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    // 뒷면 컬링 비활성화 (법선 방향 상관없이 렌더)
    ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
    // depth test 명시적 활성화
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    // Clampable overlays are rendered at a much higher bin (200000). Keep terrain explicit.
    ss->setRenderBinDetails(0, "RenderBin");
    // 오버레이 유니폼 기본값 초기화 (attach 전까지 비활성)
    initHyperTerrainLitStateSet(ss);

    auto* geode = new osg::Geode();
    geode->addDrawable(geom);

    // ---- MatrixTransform: prepareInLoadThread transform * glTF node matrix ----
    glm::dmat4 worldTransform(1.0);
    if (pLoadThreadResult) {
        worldTransform = static_cast<const LoadThreadData*>(pLoadThreadResult)->transform;
    }
    if (!model.nodes.empty()) {
        const auto& node = model.nodes[0];
        if (node.matrix.size() == 16) {
            // glTF는 column-major 저장: matrix[0..3]=col0, matrix[4..7]=col1, ..., matrix[12..15]=col3(translation)
            // glm::dmat4(scalar...) 생성자: 첫 4개 = col0, 다음 4개 = col1, ...  (column-major)
            glm::dmat4 nodeMat(
                node.matrix[0],  node.matrix[1],  node.matrix[2],  node.matrix[3],
                node.matrix[4],  node.matrix[5],  node.matrix[6],  node.matrix[7],
                node.matrix[8],  node.matrix[9],  node.matrix[10], node.matrix[11],
                node.matrix[12], node.matrix[13], node.matrix[14], node.matrix[15]);
            worldTransform = worldTransform * nodeMat;
        } else if (node.translation.size() == 3) {
            // matrix 없이 TRS decomposition
            worldTransform[3] = glm::dvec4(node.translation[0], node.translation[1], node.translation[2], 1.0);
        }
    }

    // ---- glTF Y-up → ECEF Z-up 변환 ----
    // cesium-native glTF 타일은 Y-up 월드 공간 기준:
    //   X_ecef = X_world
    //   Y_ecef = -Z_world
    //   Z_ecef = Y_world
    // glm M*v column-major: col0=(1,0,0,0), col1=(0,0,1,0), col2=(0,-1,0,0), col3=(0,0,0,1)
    static const glm::dmat4 kYUpToZUp(
        1.0,  0.0, 0.0, 0.0,  // col0
        0.0,  0.0, 1.0, 0.0,  // col1
        0.0, -1.0, 0.0, 0.0,  // col2
        0.0,  0.0, 0.0, 1.0); // col3
    worldTransform = kYUpToZUp * worldTransform;

    // ---- glm::dmat4 → osg::Matrixd 변환 ----
    // glm: column-major 저장, M*v (열벡터)
    // OSG: row-major 저장, v*M (행벡터)
    // 같은 16개 더블 메모리를 그대로 쓰면 수학적으로 동등한 변환 → glm::value_ptr 사용
    const double* p = glm::value_ptr(worldTransform);
    osg::Matrixd osgMat(
        p[0],  p[1],  p[2],  p[3],
        p[4],  p[5],  p[6],  p[7],
        p[8],  p[9],  p[10], p[11],
        p[12], p[13], p[14], p[15]);

    auto* xformNode = new osg::MatrixTransform(osgMat);
    xformNode->addChild(geode);

    if (m_sceneRoot.valid()) {
        m_sceneRoot->addChild(xformNode);
    }

    HyperTerrainTileRenderData data;
    data.xform = xformNode;
    data.geom = geom;
    HyperTerrainTileRenderHandle* handle = allocateHandle(data);
    if (pLoadThreadResult) delete static_cast<LoadThreadData*>(pLoadThreadResult);
    return handle;
}

void HyperTerrainPrepareRendererResources::free(
    Cesium3DTilesSelection::Tile& /*tile*/,
    void* pLoadThreadResult,
    void* pMainThreadResult) noexcept
{
    if (pLoadThreadResult) {
        delete static_cast<LoadThreadData*>(pLoadThreadResult);
    }

    if (!pMainThreadResult) return;
    auto* handle = static_cast<HyperTerrainTileRenderHandle*>(pMainThreadResult);
    HyperTerrainTileRenderData data;
    const bool resolved = resolveHandle(handle, data);

    if (resolved && m_sceneRoot.valid() && data.xform.valid()) {
        m_sceneRoot->removeChild(data.xform);
    }
    if (resolved && data.geom.valid()) {
        osg::StateSet* ss = data.geom->getStateSet();
        std::lock_guard<std::mutex> lock(m_overlayMutex);
        m_rasterAttachments.erase(
            std::remove_if(
                m_rasterAttachments.begin(),
                m_rasterAttachments.end(),
                [&](const RasterAttachment& attachment) {
                    return attachment.stateSet.get() == ss;
                }),
            m_rasterAttachments.end());
    }

    releaseHandle(handle);
}

// ---- RasterOverlay 위임 ----

void* HyperTerrainPrepareRendererResources::prepareRasterInLoadThread(
    CesiumGltf::ImageAsset& image,
    const std::any& rendererOptions)
{
    HyperTerrainRasterRenderer rasterRenderer;
    return rasterRenderer.prepareRasterInLoadThread(image, rendererOptions);
}

void* HyperTerrainPrepareRendererResources::prepareRasterInMainThread(
    CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pLoadThreadResult)
{
    HyperTerrainRasterRenderer rasterRenderer;
    return rasterRenderer.prepareRasterInMainThread(rasterTile, pLoadThreadResult);
}

void HyperTerrainPrepareRendererResources::freeRaster(
    const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pLoadThreadResult,
    void* pMainThreadResult) noexcept
{
    HyperTerrainRasterRenderer rasterRenderer;
    rasterRenderer.freeRaster(rasterTile, pLoadThreadResult, pMainThreadResult);
}

void HyperTerrainPrepareRendererResources::attachRasterInMainThread(
    const Cesium3DTilesSelection::Tile& tile,
    int32_t overlayTextureCoordinateID,
    const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pMainThreadRendererResources,
    const glm::dvec2& translation,
    const glm::dvec2& scale)
{
    if (!pMainThreadRendererResources) return;
    HyperTerrainTileRenderData tileData;
    if (!tryResolveTileRenderData(tile, tileData) || !tileData.geom.valid()) {
        OSG_WARN << "[HyperTerrain] attachRasterInMainThread: invalid tile render data." << std::endl;
        return;
    }

    const int cesiumTcId = std::max(0, std::min(static_cast<int>(overlayTextureCoordinateID), kMaxOverlays - 1));
    auto* tex = static_cast<osg::Texture2D*>(pMainThreadRendererResources);
    osg::StateSet* ss = tileData.geom->getOrCreateStateSet();

    // 동일 textureCoordinateID(대개 0)를 쓰는 베이스 TMS/위성 + OWM 구름 등이 겹치면
    // 같은 OSG 유닛에 연달아 바인딩되어 먼저 붙은 타일이 사라진다. 빈 유닛으로 분산한다.
    int slot = cesiumTcId;
    auto* existing = dynamic_cast<osg::Texture2D*>(
        ss->getTextureAttribute(cesiumTcId, osg::StateAttribute::TEXTURE));
    if (existing != nullptr && existing != tex) {
        slot = -1;
        for (int j = 0; j < kMaxOverlays; ++j) {
            auto* ej = dynamic_cast<osg::Texture2D*>(ss->getTextureAttribute(j, osg::StateAttribute::TEXTURE));
            if (ej == nullptr) {
                slot = j;
                break;
            }
        }
        if (slot < 0) {
            OSG_WARN << "[HyperTerrain] attachRasterInMainThread: all " << kMaxOverlays
                     << " overlay texture units in use; cannot attach raster." << std::endl;
            return;
        }
    }

    ss->setTextureAttributeAndModes(slot, tex, osg::StateAttribute::ON);
    if (auto* u = ss->getUniform("u_overlayTex"))    u->setElement(slot, slot);
    if (auto* u = ss->getUniform("u_overlayTrans"))   u->setElement(slot,
        osg::Vec2f(static_cast<float>(translation.x), static_cast<float>(translation.y)));
    if (auto* u = ss->getUniform("u_overlayScale"))   u->setElement(slot,
        osg::Vec2f(static_cast<float>(scale.x), static_cast<float>(scale.y)));
    if (auto* u = ss->getUniform("u_overlayAlpha"))   u->setElement(slot,
        rasterOverlayAlpha(&rasterTile.getOverlay()));
    if (auto* u = ss->getUniform("u_overlayActive"))  u->setElement(slot, 1);

    {
        std::lock_guard<std::mutex> lock(m_overlayMutex);
        m_rasterAttachments.push_back(
            RasterAttachment{&rasterTile.getOverlay(), ss, tex, slot});
    }
}

void HyperTerrainPrepareRendererResources::detachRasterInMainThread(
    const Cesium3DTilesSelection::Tile& tile,
    int32_t overlayTextureCoordinateID,
    const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
    void* pMainThreadRendererResources) noexcept
{
    HyperTerrainTileRenderData tileData;
    if (!tryResolveTileRenderData(tile, tileData) || !tileData.geom.valid()) return;

    osg::StateSet* ss = tileData.geom->getOrCreateStateSet();

    if (pMainThreadRendererResources) {
        auto* tex = static_cast<osg::Texture2D*>(pMainThreadRendererResources);
        for (int s = 0; s < kMaxOverlays; ++s) {
            auto* bound = dynamic_cast<osg::Texture2D*>(
                ss->getTextureAttribute(s, osg::StateAttribute::TEXTURE));
            if (bound == tex) {
                ss->removeTextureAttribute(s, osg::StateAttribute::TEXTURE);
                if (auto* u = ss->getUniform("u_overlayActive"))
                    u->setElement(s, 0);
                {
                    std::lock_guard<std::mutex> lock(m_overlayMutex);
                    m_rasterAttachments.erase(
                        std::remove_if(
                            m_rasterAttachments.begin(),
                            m_rasterAttachments.end(),
                            [&](const RasterAttachment& attachment) {
                                return attachment.overlay == &rasterTile.getOverlay()
                                    && attachment.stateSet.get() == ss
                                    && attachment.slot == s;
                            }),
                        m_rasterAttachments.end());
                }
                return;
            }
        }
    }

    const int slot = std::min(static_cast<int>(overlayTextureCoordinateID), kMaxOverlays - 1);
    ss->removeTextureAttribute(slot, osg::StateAttribute::TEXTURE);
    if (auto* u = ss->getUniform("u_overlayActive"))  u->setElement(slot, 0);
    {
        std::lock_guard<std::mutex> lock(m_overlayMutex);
        m_rasterAttachments.erase(
            std::remove_if(
                m_rasterAttachments.begin(),
                m_rasterAttachments.end(),
                [&](const RasterAttachment& attachment) {
                    return attachment.overlay == &rasterTile.getOverlay()
                        && attachment.stateSet.get() == ss
                        && attachment.slot == slot;
                }),
            m_rasterAttachments.end());
    }
}

} // namespace Cesium
} // namespace osgEarth
